from pymavlink import mavutil
# from .mavsub import wifi as mavwifi
import time
import threading
import socket

class EdubotGCS(): #mavwifi.Wifi
    """Ground Command System (PC) class"""
    MAV_RESULT = {
        -1: 'SEND_TIMEOUT',
        0: 'ACCEPTED',
        1: 'TEMPORARILY_REJECTED',
        2: 'DENIED',
        3: 'UNSUPPORTED',
        4: 'FAILED',
        5: 'IN_PROGRESS',
        6: 'CANCELLED'
    }

    AUTOPILOT_STATE = {
    }

    _SUPPORTED_CONNECTION_METHODS = ['udpout', 'serial']

    def __init__(self, name='edubotGCS', ip='localhost', mavlink_port=8001, connection_method='udpout',
                 device='/dev/serial0', baud=115200,
                 logger=True, log_connection=True):


        self.name = name

        self._is_connected = False
        self._is_connected_timeout = 1
        self._last_msg_time = time.time() - self._is_connected_timeout

        self._heartbeat_timeout = 1
        self._heartbeat_send_time = time.time() - self._heartbeat_timeout

        self._mavlink_send_timeout = 0.5
        self._mavlink_send_long_timeout = 1
        self._mavlink_send_number = 10

        self._logger = logger
        self._log_connection = log_connection

        self._point_seq = 0
        self._point_reached = False

        self._cur_state = None
        self._preride_state = dict(BatteryLow=None,            #todo
                                     NavSystem=None,
                                     Area=None,
                                     Attitude=None,
                                     RcExpected=None,
                                     RcMode=None,
                                     RcUnexpected=None,
                                     UavStartAllowed=None)

        self.mavlink_socket = self._create_connection(connection_method=connection_method,
                                                      ip=ip, port=mavlink_port,
                                                      device=device, baud=baud)
        self.__is_socket_open = threading.Event()  # Flag for the concurrent thread. Signals whether or not the thread should go on running
        self.__is_socket_open.set()

        self.msg_archive = dict()
        self.wait_msg = dict()

        self._message_handler_thread = threading.Thread(target=self._message_handler, daemon=True)
        self._message_handler_thread.daemon = True
        self._message_handler_thread.start()

        self.log(msg_type='connection', msg='Connecting to car...')
        # mavwifi.Wifi.__init__(self, self.mavlink_socket)

    def __del__(self):
        self.log(msg="Edubot class object removed")

    def _create_connection(self, connection_method, ip, port, device, baud):
        """
        create mavlink connection
        :return: mav_socket
        """
        if connection_method not in self._SUPPORTED_CONNECTION_METHODS:
            raise ValueError(f"Unknown connection method: {connection_method}")

        mav_socket = None
        try:
            if connection_method == "serial":
                mav_socket = mavutil.mavlink_connection(device=device, baud=baud)
            else:
                mav_socket = mavutil.mavlink_connection('%s:%s:%s' % (connection_method, ip, port))

            return mav_socket

        except socket.error as e:
            raise ConnectionError('Connection error. Can not connect to drone', e)

    def close_connection(self):
        """
        Close mavlink connection
        :return: None
        """
        self.__is_socket_open.clear()
        self._message_handler_thread.join()
        self.mavlink_socket.close()

        self.log(msg_type='connection', msg='Close mavlink socket')

    def log(self, msg, msg_type=None):
        if msg_type == "connection" and not self._log_connection:
            return
        elif msg_type != "connection" and not self._logger:
            return

        if msg_type is None:
            print(f"[{self.name}] {msg}")
        else:
            print(f"[{self.name}] <{msg_type}> {msg}")

    def connected(self):
        return self._is_connected

    def set_logger(self, value: bool = True):
        self._logger = value

    def set_log_connection(self, value: bool = True):
        self._log_connection = value

    def _send_heartbeat(self):
        self.mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                               mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self._heartbeat_send_time = time.time()

    def _receive_heartbeat(self, msg):
        try:
            if msg._header.srcComponent == 1:
                custom_mode = msg.custom_mode
                custom_mode_buf = format(custom_mode, "032b")
                status_autopilot = custom_mode_buf[24:]
                self._cur_state = EdubotGCS.AUTOPILOT_STATE[int(status_autopilot, 2)] # todo состояние
        except Exception:
            pass

    def _mission_item_reached(self, msg):
        if self._point_seq is None:
            self._point_seq = msg.seq
        if msg.seq > self._point_seq:
            self._point_reached = True

            self.log(msg_type='POINT_REACHED', msg=f'point_id: {msg.seq}')
        self._point_seq = msg.seq

    def _message_handler(self):
        while True:
            if not self.__is_socket_open.is_set():
                break

            if time.time() - self._heartbeat_send_time >= self._heartbeat_timeout:
                self._send_heartbeat()

            msg = self.mavlink_socket.recv_msg()
            if msg is not None:
                self._last_msg_time = time.time()
                if not self._is_connected:
                    self._is_connected = True
                    self.log(msg_type='connection', msg='CONNECTED')
                if msg.get_type() == 'HEARTBEAT':
                    self._receive_heartbeat(msg)
                elif msg.get_type() == 'MISSION_ITEM_REACHED':
                    self._mission_item_reached(msg)
                elif msg.get_type() == 'COMMAND_ACK':
                    msg._type += f'_{msg.command}'
                    if msg.command == 400 and msg.result_param2 is not None:
                        self._preflight_state.update(BatteryLow=msg.result_param2 & 0b00000001)
                        self._preflight_state.update(NavSystem=msg.result_param2 & 0b00000010)
                        self._preflight_state.update(Area=msg.result_param2 & 0b00000100)
                        self._preflight_state.update(Attitude=msg.result_param2 & 0b00001000)
                        self._preflight_state.update(RcExpected=msg.result_param2 & 0b00010000)
                        self._preflight_state.update(RcMode=msg.result_param2 & 0b00100000)
                        self._preflight_state.update(RcUnexpected=msg.result_param2 & 0b01000000)
                        self._preflight_state.update(UavStartAllowed=msg.result_param2 & 0b10000000)

                if msg.get_type() in self.wait_msg:
                    self.wait_msg[msg.get_type()].set()
                self.msg_archive.update({msg.get_type(): {'msg': msg, 'is_read': threading.Event()}})

            elif self._is_connected and (time.time() - self._last_msg_time > self._is_connected_timeout):
                self._is_connected = False
                self.log(msg_type='connection', msg='DISCONNECTED')

        self.log(msg="Message handler stopped")

    def _send_command_long(self, command_name, command, param1: float = 0, param2: float = 0, param3: float = 0,
                           param4: float = 0, param5: float = 0, param6: float = 0, param7: float = 0,
                           target_system=None, target_component=None, sending_log_msg='sending...'):

        self.log(msg_type=command_name, msg=sending_log_msg)
        if target_system is None:
            target_system = self.mavlink_socket.target_system
        if target_component is None:
            target_component = self.mavlink_socket.target_component
        if_send = True
        in_progress = False
        confirm = 0
        msg_to_wait = f'COMMAND_ACK_{command}'
        event = threading.Event()
        self.wait_msg[msg_to_wait] = event
        try:
            while True:
                if if_send:
                    self.mavlink_socket.mav.command_long_send(target_system, target_component, command, confirm,
                                                              param1, param2, param3, param4, param5, param6, param7)
                    confirm += 1

                if in_progress:
                    event.wait(self._mavlink_send_long_timeout)
                else:
                    event.wait(self._mavlink_send_timeout)

                if event.is_set():
                    if_send = False
                    msg = self.msg_archive[msg_to_wait]['msg']
                    self.msg_archive[msg_to_wait]['is_read'].set()
                    if msg.result == 5:  # IN_PROGRESS
                        in_progress = True

                        self.log(msg_type=command_name, msg=EdubotGCS.MAV_RESULT[msg.result])
                        event.clear()
                    else:

                        self.log(msg_type=command_name, msg=EdubotGCS.MAV_RESULT[msg.result])
                        return msg.result in [0, 2]
                else:
                    if_send = True
                if confirm >= self._mavlink_send_number:
                    self.log(msg_type=command_name, msg=EdubotGCS.MAV_RESULT[-1])
                    return False
        finally:
            if msg_to_wait in self.wait_msg:
                del self.wait_msg[msg_to_wait]

    def _send_position_target_local_ned(self, command_name, coordinate_system, mask=0b0000_11_0_111_111_111, x=0, y=0,
                                        z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0,
                                        target_system=None, target_component=None):
        if target_system is None:
            target_system = self.mavlink_socket.target_system
        if target_component is None:
            target_component = self.mavlink_socket.target_component
        event = threading.Event()
        self.wait_msg['POSITION_TARGET_LOCAL_NED'] = event
        try:
            for confirm in range(self._mavlink_send_number):
                self.mavlink_socket.mav.set_position_target_local_ned_send(0, target_system, target_component,
                                                                           coordinate_system,
                                                                           mask, x, y, z, vx, vy, vz, afx, afy, afz,
                                                                           yaw, yaw_rate)
                event.wait(self._mavlink_send_timeout)
                if event.is_set():
                    msg = self.msg_archive['POSITION_TARGET_LOCAL_NED']['msg']
                    self.msg_archive['POSITION_TARGET_LOCAL_NED']['is_read'].set()
                    if msg.type_mask == mask:

                        self.log(msg_type=command_name, msg=EdubotGCS.MAV_RESULT[0])
                    else:

                        self.log(msg_type=command_name, msg=EdubotGCS.MAV_RESULT[2])
                    return True

            self.log(msg_type=command_name, msg=EdubotGCS.MAV_RESULT[-1])
            return False
        finally:
            if 'POSITION_TARGET_LOCAL_NED' in self.wait_msg:
                del self.wait_msg['POSITION_TARGET_LOCAL_NED']


    def go_to_local_point(self, x, y, z, yaw):
        """ Flight to point in the current navigation system's coordinate frame """
        cmd_name = 'GO_TO_POINT'

        self.log(msg_type=cmd_name, msg=f'sending point {{LOCAL, x:{x}, y:{y}, z:{z}, yaw:{yaw}}} ...')
        mask = 0b0000_10_0_111_111_000  # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        x, y, z = y, x, -z  # ENU coordinates to NED coordinates
        self._point_reached = False
        return self._send_position_target_local_ned(command_name=cmd_name,
                                                    coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                    mask=mask, x=x, y=y, z=z, yaw=yaw)

    def go_to_local_point_body_fixed(self, x, y, z, yaw):
        """ Flight to point relative to the current position """

        cmd_name = 'GO_TO_POINT'

        self.log(msg_type=cmd_name, msg=f'sending point {{BODY_FIX, x:{x}, y:{y}, z:{z}, yaw:{yaw}}} ...')
        mask = 0b0000_10_0_111_111_000  # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        x, y, z = y, x, -z  # ENU coordinates to NED coordinates
        self._point_reached = False
        return self._send_position_target_local_ned(command_name=cmd_name,
                                                    coordinate_system=mavutil.mavlink.MAV_FRAME_BODY_FRD,
                                                    mask=mask, x=x, y=y, z=z, yaw=yaw)

    def point_reached(self):
        if self._point_reached:
            self._point_reached = False
            return True
        else:
            return False

    def get_local_position_lps(self, get_last_received: bool = False):
        """ Get position LPS"""
        if 'LOCAL_POSITION_NED' in self.msg_archive:
            msg_dict = self.msg_archive['LOCAL_POSITION_NED']
            msg = msg_dict['msg']
            if not msg_dict['is_read'].is_set() or (msg_dict['is_read'].is_set() and get_last_received):
                msg_dict['is_read'].set()
                return [msg.x, msg.y, msg.z]
            else:
                return None
        else:
            return None

    def get_battery_status(self, get_last_received: bool = False):
        """ Returning battery remaining voltage """
        if 'BATTERY_STATUS' in self.msg_archive:
            msg_dict = self.msg_archive['BATTERY_STATUS']
            if not msg_dict['is_read'].is_set() or (msg_dict['is_read'].is_set() and get_last_received):
                msg_dict['is_read'].set()
                return msg_dict['msg'].voltages[0] / 100
            else:
                return None
        else:
            return None


    def get_autopilot_state(self):
        return self._cur_state

    def send_rc_channels(self, channel_1=0xFF, channel_2=0xFF, channel_3=0xFF, channel_4=0xFF,
                         channel_5=0xFF, channel_6=0xFF, channel_7=0xFF, channel_8=0xFF):
        """ RC unit signal send to drone """
        self.mavlink_socket.mav.rc_channels_override_send(self.mavlink_socket.target_system,
                                                          self.mavlink_socket.target_component, channel_1,
                                                          channel_2, channel_3, channel_4, channel_5, channel_6,
                                                          channel_7, channel_8)

    def raspberry_poweroff(self):
        return self._send_command_long(command_name='RPi_POWEROFF',
                                       command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       target_component=42)

    def raspberry_reboot(self):
        return self._send_command_long(command_name='RPi_REBOOT',
                                       command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       target_component=43)



