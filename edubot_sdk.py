import dataclasses
import enum
import socket
import threading

from pymavlink import mavutil
from pymavlink.dialects.v20.common import *


class EdubotGCS:  # mavwifi.Wifi
    """Ground Command System (PC) class"""

    class ConnectionMethod(enum.Enum):
        updin = 0
        udpout = 1
        serial = 2

    class __MavResult(enum.Enum):
        SEND_TIMEOUT = -1
        ACCEPTED = MAV_RESULT_ACCEPTED
        TEMPORARILY_REJECTED = MAV_RESULT_TEMPORARILY_REJECTED
        DENIED = MAV_RESULT_DENIED
        UNSUPPORTED = MAV_RESULT_UNSUPPORTED
        FAILED = MAV_RESULT_FAILED
        IN_PROGRESS = MAV_RESULT_IN_PROGRESS
        CANCELLED = 6

    @dataclasses.dataclass
    class __MavSettings:
        connected_timeout: float = 1  # таймаут для принятия решения о потере соединения
        heartbeat_timeout: float = 1  # таймаут для отправки heartbeat
        mavlink_send_timeout: float = 0.5
        mavlink_send_long_timeout: float = 1
        mavlink_send_number: int = 10  # кол-во попыток отпрвки command_long

    logging.basicConfig(level=logging.CRITICAL, stream=sys.stdout)

    def __init__(self, ip='localhost', mavlink_port=8001, connectionMethod=ConnectionMethod.udpout,
                 device='/dev/serial0', baud=115200, name='EdubotStation'):

        self.name = name

        self.__mav_settings = self.__MavSettings()

        self._is_connected = False
        self._last_msg_time = time.time() - self.__mav_settings.connected_timeout
        self._heartbeat_send_time = time.time() - self.__mav_settings.heartbeat_timeout

        self._point_seq = 0
        self._point_reached = False

        self.mavlink_socket = self._create_connection(connectionMethod=connectionMethod,
                                                      ip=ip, port=mavlink_port,
                                                      device=device, baud=baud)
        self.__is_socket_open = threading.Event()  # Flag for the concurrent thread. Signals whether or not the thread should go on running
        self.__is_socket_open.set()

        self.msg_archive = dict()
        self.wait_msg = dict()

        self._message_handler_thread = threading.Thread(target=self._message_handler, daemon=True)
        self._message_handler_thread.daemon = True
        self._message_handler_thread.start()

        logging.info(f"[{self.name}] <Connection> connecting to car...")

    def __del__(self):
        logging.debug(f"{self.name}:object: Class object removed")

    def _create_connection(self, connectionMethod, ip, port, device, baud):
        """
        create mavlink connection
        :return: mav_socket
        """
        if connectionMethod not in self.ConnectionMethod:
            logging.error(f"{self.name}:Connection: Unknown connection method: {connectionMethod}")

        mav_socket = None
        try:
            if connectionMethod == self.ConnectionMethod.serial:
                mav_socket = mavutil.mavlink_connection(device=device, baud=baud)
            else:
                mav_socket = mavutil.mavlink_connection('%s:%s:%s' % (connectionMethod.name, ip, port))

            return mav_socket

        except socket.error as e:
            logging.error(f"{self.name}:Connection: Can not connect to robot: {e}")

    def close_connection(self):
        """
        Close mavlink connection
        :return: None
        """
        self.__is_socket_open.clear()
        self._message_handler_thread.join()
        self.mavlink_socket.close()
        logging.info(f"[{self.name}] <Connection> Mavlink socket closed")

    def connected(self):
        return self._is_connected

    def _send_heartbeat(self):
        self.mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                               mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self._heartbeat_send_time = time.time()

    def _mission_item_reached(self, msg):
        if self._point_seq is None:
            self._point_seq = msg.seq
        if msg.seq > self._point_seq:
            self._point_reached = True
            logging.info(f"[{self.name}] <Reached> point_id {msg.seq}")
        self._point_seq = msg.seq

    def _message_handler(self):
        while True:
            if not self.__is_socket_open.is_set():
                break

            if time.time() - self._heartbeat_send_time >= self.__mav_settings.heartbeat_timeout:
                self._send_heartbeat()

            msg = self.mavlink_socket.recv_msg()
            if msg is not None:
                self._last_msg_time = time.time()
                if not self._is_connected:
                    self._is_connected = True

                if msg.id == MAVLINK_MSG_ID_HEARTBEAT:
                    pass
                elif msg.id == MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
                    self._mission_item_reached(msg)
                elif msg.id == MAVLINK_MSG_ID_COMMAND_ACK:
                    msg._type += f'_{msg.command}'

                if msg.get_type() in self.wait_msg:
                    self.wait_msg[msg.get_type()].set()
                self.msg_archive.update({msg.get_type(): {'msg': msg, 'is_read': threading.Event()}})

            elif self._is_connected and (time.time() - self._last_msg_time > self.__mav_settings.connected_timeout):
                self._is_connected = False
                logging.info(f"[{self.name}] <Connection> disconnected")

        logging.info(f"[{self.name}] <Object> message handler stopped")

    def _send_command_long(self, command_name, command, param1: float = 0, param2: float = 0, param3: float = 0,
                           param4: float = 0, param5: float = 0, param6: float = 0, param7: float = 0,
                           target_system=None, target_component=None):

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
                    event.wait(self.__mav_settings.mavlink_send_long_timeout)
                else:
                    event.wait(self.__mav_settings.mavlink_send_timeout)

                if event.is_set():
                    if_send = False
                    msg = self.msg_archive[msg_to_wait]['msg']
                    self.msg_archive[msg_to_wait]['is_read'].set()

                    if msg.result == MAV_RESULT_IN_PROGRESS:  # IN_PROGRESS
                        in_progress = True
                        logging.debug(
                            f"[{self.name}] <Ack> {command_name}, result: {self.__MavResult(msg.result).name}")
                        event.clear()
                    else:
                        logging.debug(
                            f"[{self.name}] <Ack> {command_name}, result: {self.__MavResult(msg.result).name}")
                        return msg.result in [MAV_RESULT_ACCEPTED, MAV_RESULT_DENIED]
                else:
                    if_send = True
                if confirm >= self.__mav_settings.mavlink_send_number:
                    logging.debug(f"[{self.name}] <Ack> {command_name}, result: {self.__MavResult.SEND_TIMEOUT.name}")
                    return False
        finally:
            if msg_to_wait in self.wait_msg:
                del self.wait_msg[msg_to_wait]

    def _send_position_target_local_ned(self, command_name, coordinate_system, x, y, mask=0b0000_11_0_111_111_111,
                                        z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0,
                                        target_system=None, target_component=None):
        if target_system is None:
            target_system = self.mavlink_socket.target_system
        if target_component is None:
            target_component = self.mavlink_socket.target_component
        event = threading.Event()
        self.wait_msg['POSITION_TARGET_LOCAL_NED'] = event
        try:
            for confirm in range(self.__mav_settings.mavlink_send_number):
                self.mavlink_socket.mav.set_position_target_local_ned_send(0, target_system, target_component,
                                                                           coordinate_system,
                                                                           mask, x, y, z, vx, vy, vz, afx, afy, afz,
                                                                           yaw, yaw_rate)
                event.wait(self.__mav_settings.mavlink_send_timeout)
                if event.is_set():
                    msg = self.msg_archive['POSITION_TARGET_LOCAL_NED']['msg']
                    self.msg_archive['POSITION_TARGET_LOCAL_NED']['is_read'].set()
                    if msg.type_mask == mask:
                        logging.debug(f"[{self.name}] <Ack> {command_name}, result: {self.__MavResult.ACCEPTED.name}")
                    else:
                        logging.debug(f"[{self.name}] <Ack> {command_name}, result: {self.__MavResult.DENIED.name}")
                    return True

            logging.debug(f"[{self.name}] <Ack> {command_name}, result: {self.__MavResult.SEND_TIMEOUT.name}")
            return False
        finally:
            if 'POSITION_TARGET_LOCAL_NED' in self.wait_msg:
                del self.wait_msg['POSITION_TARGET_LOCAL_NED']

    def go_to_local_point(self, x, y):
        """ Поездка в точку с глобальными координатами """
        cmd_name = 'GO_TO_POINT'
        logging.info(f"[{self.name}] <Point> target: local point {{x:{x}, y:{y}}}...")
        mask = 0b0000_10_0_111_111_000  # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        self._point_reached = False
        return self._send_position_target_local_ned(command_name=cmd_name,
                                                    coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_ENU,
                                                    mask=mask, x=x, y=y)

    def go_to_local_point_body_fixed(self, x, y):
        """ Поездка в точку с координатами, заданными относительно текущих """
        cmd_name = 'GO_TO_POINT_BODY_FIXED'
        logging.info(f"[{self.name}] <Point> target: body_fixed point {{x:{x}, y:{y}}}...")
        mask = 0b0000_10_0_111_111_000  # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        self._point_reached = False
        return self._send_position_target_local_ned(command_name=cmd_name,
                                                    coordinate_system=mavutil.mavlink.MAV_FRAME_BODY_FRD,
                                                    mask=mask, x=x, y=y)

    def point_reached(self):
        """ Была ли достигнута предыдущая заданная точка """
        if self._point_reached:
            self._point_reached = False
            return True
        else:
            return False

    def get_local_position_lps(self, get_last_received: bool = False):
        """ Возвращает текущие координаты робота [x, y, z] """
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
        """ Возвращает значение напряжения с батареи робота """
        if 'BATTERY_STATUS' in self.msg_archive:
            msg_dict = self.msg_archive['BATTERY_STATUS']
            if not msg_dict['is_read'].is_set() or (msg_dict['is_read'].is_set() and get_last_received):
                msg_dict['is_read'].set()
                return msg_dict['msg'].voltages[0] / 100
            else:
                return None
        else:
            return None

    def get_attitude(self, get_last_received: bool = False):
        """ Возвращает значение угла рыскания робота """
        if 'ATTITUDE' in self.msg_archive:
            msg_dict = self.msg_archive['ATTITUDE']
            msg = msg_dict['msg']
            if not msg_dict['is_read'].is_set() or (msg_dict['is_read'].is_set() and get_last_received):
                msg_dict['is_read'].set()
                return msg.yaw
            else:
                return None
        else:
            return None

    def _rc_channels_send(self, channel_1=0xFF, channel_2=0xFF, channel_3=0xFF, channel_4=0xFF,
                          channel_5=0xFF, channel_6=0xFF, channel_7=0xFF, channel_8=0xFF):
        self.mavlink_socket.mav.rc_channels_override_send(self.mavlink_socket.target_system,
                                                          self.mavlink_socket.target_component, channel_1,
                                                          channel_2, channel_3, channel_4, channel_5, channel_6,
                                                          channel_7, channel_8)

    def raspberry_poweroff_send(self):
        """ Выключить робота """
        return self._send_command_long(command_name='RPi_POWEROFF',
                                       command=MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       param1=0)

    def raspberry_reboot_send(self):
        """ Перезагрузить робота """
        return self._send_command_long(command_name='RPi_REBOOT',
                                       command=MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       param1=1)

    def set_servo(self, num_servo, pwm):
        """ Управление сервой  """
        return self._send_command_long(command_name='RPi_REBOOT',
                                       command=MAV_CMD_DO_SET_SERVO,
                                       param1=pwm,
                                       param2=num_servo)


    def led_control(self, r=0, g=0, b=0, led_id=255):
        """ Установить цвет подсветки  """
        max_value = 255.0
        all_led = 255
        first_led = 0
        last_led = 3
        led_value = [r, g, b]
        command = True

        try:
            if led_id != all_led and (led_id < first_led or led_id > last_led):
                command = False
            for i in range(len(led_value)):
                led_value[i] = float(led_value[i])
                if led_value[i] > max_value or led_value[i] < 0:
                    command = False
                    break
        except ValueError:
            command = False

        if command:
            if led_id == all_led:
                led_id_print = 'all'
            else:
                led_id_print = led_id

            logging.info(f"{self.name} <LED> id: %s R: %i ,G: %i, B: %i send" % (led_id_print, r, g, b))
            self._send_command_long(
                0, # command_name
                mavutil.mavlink.MAV_CMD_USER_1,  # command
                param1=led_id,  # param1
                param2=led_value[0],  # param2
                param3=led_value[1],  # param3
                param4=led_value[2] , # param4
                target_system = self.mavlink_socket.target_system,
                target_component = self.mavlink_socket.target_component
                )
        else:
            logging.error(f"{self.name} <LED> Wrong LED values")

    def led_custom(self, mode=1, timer=0, color1=(0, 0, 0), color2=(0, 0, 0)):
        param2 = (((color1[0] << 8) | color1[1]) << 8) | color1[2]
        param3 = (((color2[0] << 8) | color2[1]) << 8) | color2[2]

        self._send_command_long(
            0,  # command name
            mavutil.mavlink.MAV_CMD_USER_3,  # command
            0,  # param1
            param2,  # param2
            param3,  # param3
            0,  # param4
            param5=mode,  # param5
            param6=timer, # param6
            target_system=self.mavlink_socket.target_system,
            target_component=self.mavlink_socket.target_component
        )
