from pymavlink import mavutil
# from .mavsub import wifi as mavwifi
import time
import threading
import socket
import edubot_waypoints



class EdubotVehicle:
    """Edubot vehicle class"""
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
    _SUPPORTED_CONNECTION_METHODS = ['serial', 'udpin']
    def __init__(self, name='edubotVehicle', ip='localhost', mavlink_port=8001, connection_method='udpin',
                 device='/dev/serial0', baud=115200,
                 logger=True, log_connection=True):

        self.name = name
        self._vehicle = edubot_waypoints.WaypointsRobot(logger)

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

        self._point_seq = None
        self._point_reached = False

        self._cur_state = None
        self._preflight_state = dict(BatteryLow=None,
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

        self.log(msg_type='connection', msg='Connecting to drone...')
        # mavwifi.Wifi.__init__(self, self.mavlink_socket)

    def __del__(self):
        self.log(msg="EdubotVehicle class object removed")
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
                mav_socket.mav.mission_item_reached_send(0)
                print('%s:%s:%s' % (connection_method, ip, port))

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
        self.mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                                               mavutil.mavlink.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, 0, 0, 0)
        self._heartbeat_send_time = time.time()

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
                elif msg.get_type() == 'COMMAND_ACK':
                    msg._type += f'_{msg.command}'
                elif msg.get_type() == 'ATTITUDE':
                    self._attitude_send()

                if msg.get_type() in self.wait_msg:
                    self.wait_msg[msg.get_type()].set()

                self.msg_archive.update({msg.get_type(): {'msg': msg, 'is_read': threading.Event()}})

            elif self._is_connected and (time.time() - self._last_msg_time > self._is_connected_timeout):
                self._is_connected = False

                self.log(msg_type='connection', msg='DISCONNECTED')

        self.log(msg="Message handler stopped")

    def _sys_status_send(self): # msgname = "SYS_STATUS"
        self.mavlink_socket.mav.sys_status_send()


    def _attitude_send(self): # msgname = "ATTITUDE"
        att = self._vehicle.get_attitude()
        self.mavlink_socket.mav.attitude_send(0, att[0], att[1], att[2], 0, 0, 0)


    def _local_position_ned_send(self, x=0, y=0, z=0, vx=0, vy=0, vz=0): #     msgname = "LOCAL_POSITION_NED"
        self.mavlink_socket.mav.local_position_ned_send(0, x, y, z, vx, vy, vz)

    def _mission_item_reached_send(self): # msgname = "MISSION_ITEM_REACHED"
        self.mavlink_socket.mav.mission_item_reached_send(self._point_seq)


    def _send_ack(self, msgid, status):
        self.mavlink_socket.mav.command_ack_send(msgid, 2)

    def _execute_command_long(self, msg):
        if_send = True
        in_progress = False



    def _go_to_local_point_body_fixed(self, msg):
        msgname = 'LOCAL_POSITION_NED'
        self._vehicle.go_to_local_point_body_fixed()
        #some realization
        #then send ack

    def _go_to_local_point(self, msg):
        self._vehicle.go_to_local_point()
        #some realization
        #then send ack

    def _raspberry_poweroff(self, msg):
        self._vehicle.exit_program()

    def _raspberry_reboot(self, msg):
