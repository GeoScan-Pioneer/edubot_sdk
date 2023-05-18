import dataclasses
import enum
from pymavlink.dialects.v20.common import *
from pymavlink import mavutil
import time
import threading
import socket
import edubot_waypoints
import logging
import led


class EdubotVehicle:
    """Edubot vehicle class"""

    class __MavResult(enum.Enum):
        SEND_TIMEOUT = -1
        ACCEPTED = MAV_RESULT_ACCEPTED
        TEMPORARILY_REJECTED = MAV_RESULT_TEMPORARILY_REJECTED
        DENIED = MAV_RESULT_DENIED
        UNSUPPORTED = MAV_RESULT_UNSUPPORTED
        FAILED = MAV_RESULT_FAILED
        IN_PROGRESS = MAV_RESULT_IN_PROGRESS
        CANCELLED = 6

    class ConnectionMethod(enum.Enum):
        updin = 0
        udpout = 1
        serial = 2

    @dataclasses.dataclass
    class __MavSettings:
        connected_timeout: float = 1  # таймаут для принятия решения о потере соединения
        heartbeat_timeout: float = 1  # таймаут для отправки heartbeat
        telemetery_timeout: float = 0.5
        point_seq_timeout: float = 0.5
        mavlink_send_timeout: float = 0.5
        mavlink_send_long_timeout: float = 1
        mavlink_send_number: int = 10  # кол-во попыток отпрвки command_long

    def __init__(self, name='EdubotVehicle', ip=None, mavlink_port=5656, connection_method=ConnectionMethod.udpout,
                 device='/dev/serial0', baud=115200, logging_level='INFO'):

        self.name = name
        self.__mav_settings = self.__MavSettings()
        if ip is None:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                s.connect(('10.255.255.255', 1))
                ip = s.getsockname()[0]
            except:
                ip = '127.0.0.1'
            finally:
                s.close()
        print(f"[{self.name}] <Connection> robot IP: {ip}")

        self._vehicle = edubot_waypoints.WaypointsRobot()
        self._led = led.Led()

        self._is_connected = False
        self._last_msg_time = time.time() - self.__mav_settings.connected_timeout

        self._heartbeat_send_time = time.time() - self.__mav_settings.heartbeat_timeout

        self._telemetery_send_time = time.time() - self.__mav_settings.telemetery_timeout

        self._point_seq = 0
        self._point_seq_send_time = time.time() - self.__mav_settings.point_seq_timeout

        self._driving_thread = None

        self.mavlink_socket = self._create_connection(connection_method=connection_method,
                                                      ip=ip, port=mavlink_port,
                                                      device=device, baud=baud)
        self.__is_socket_open = threading.Event()
        self.__is_socket_open.set()

        self.msg_archive = dict()
        self.wait_msg = dict()

        self.command_long_handler = {
            MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: lambda msg: self.__handler_raspberry_reboot_shutdown(msg),
            MAV_CMD_DO_SET_SERVO             : lambda msg: self.__handler_cmd_do_set_servo(msg),
            MAV_CMD_USER_1                   : lambda msg: self.__handler_led(msg),
            MAV_CMD_USER_3                   : lambda msg: self.__handler_led(msg)
        }

        self._message_handler_thread = threading.Thread(
                target=self._message_handler, daemon=True)
        self._message_handler_thread.daemon = True
        self._message_handler_thread.start()

        logging.info(f"[{self.name}] <Connection> connecting to station...")

        self.cur_point = []

    def __del__(self):
        logging.debug(f"[{self.name}] <Object> Class object removed")

    def _create_connection(self, connection_method, ip, port, device, baud):
        """
        create mavlink connection
        :return: mav_socket
        """
        if connection_method not in self.ConnectionMethod:
            logging.error(
                    f"[{self.name}] <Connection> Unknown connection method: {connection_method}")

        mav_socket = None
        try:
            if connection_method == "serial":
                mav_socket = mavutil.mavlink_connection(
                        device=device, baud=baud)
            else:
                mav_socket = mavutil.mavlink_connection(
                        '%s:%s:%s' % (connection_method, ip, port))
                mav_socket.mav.mission_item_reached_send(0)

            return mav_socket

        except socket.error as e:
            logging.error(
                    f"[{self.name}] <Connection> Connection error. Can not connect to station: {e}")

    def close_connection(self):
        """
        Close mavlink connection
        :return: None
        """
        self.__is_socket_open.clear()
        self._message_handler_thread.join()
        self.mavlink_socket.close()
        logging.error(f"[{self.name}] <Connection> Mavlink socket closed")

    def connected(self):
        """
        Check if vehicle is connected
        :return: Bool
        """
        return self._is_connected

    def _send_heartbeat(self):
        self.mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
                                               mavutil.mavlink.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, 0, 0, 0)
        self._heartbeat_send_time = time.time()

    def _message_handler(self):
        while True:
            if not self.__is_socket_open.is_set():
                break

            if time.time() - self._heartbeat_send_time >= self.__mav_settings.heartbeat_timeout:
                self._send_heartbeat()
            if time.time() - self._telemetery_send_time >= self.__mav_settings.telemetery_timeout:
                self._attitude_send()
                self._local_position_ned_send()
            if time.time() - self._point_seq_send_time >= self.__mav_settings.point_seq_timeout:
                self._mission_item_reached_send()

            msg = self.mavlink_socket.recv_msg()
            if msg is not None:
                if msg.id == MAVLINK_MSG_ID_COMMAND_LONG:
                    if msg.command in self.command_long_handler:
                        self._send_ack(msg.command, self.__MavResult.ACCEPTED)
                        self.command_long_handler[msg.command](msg)
                    else:
                        self._send_ack(msg.command, self.__MavResult.DENIED)

                self._last_msg_time = time.time()
                if not self._is_connected:
                    self._is_connected = True
                    logging.info(
                            f"[{self.name}] <Connection> connected to station")

                if msg.id == MAVLINK_MSG_ID_HEARTBEAT:
                    pass

                elif msg.id == MAVLINK_MSG_ID_COMMAND_ACK:
                    msg._type += f'_{msg.command}'

                elif msg.id == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:  # команда ехать в точку
                    self.__handler_go_to_local_point(msg)

                if msg.get_type() in self.wait_msg:  # если находится в листе ожидания (например, ack)
                    self.wait_msg[msg.get_type()].set()  # триггерим Event

                self.msg_archive.update(
                        {msg.get_type(): {'msg': msg, 'is_read': threading.Event()}})

            elif self._is_connected and (time.time() - self._last_msg_time > self.__mav_settings.connected_timeout):
                self._is_connected = False
                logging.info(f"[{self.name}] <Connection> disconnected")

        logging.info(f"[{self.name}] <Object> message handler stopped")

    def _sys_status_send(self):  # msgname = "SYS_STATUS"
        self.mavlink_socket.mav.sys_status_send()

    def _battery_status_send(self):
        voltages = self._vehicle.get_battery_status()
        self.mavlink_socket.mav.battery_status_send(0, 0, 0, 0, voltages, 0, 0, 0, 0)

    def _attitude_send(self):
        att = self._vehicle.get_attitude()
        self.mavlink_socket.mav.attitude_send(
                0, att[0], att[1], att[2], 0, 0, 0)
        self._telemetery_send_time = time.time()

    def _local_position_ned_send(self):
        pos = self._vehicle.get_local_position()
        self.mavlink_socket.mav.local_position_ned_send(
                0, pos[0], pos[1], pos[2], 0, 0, 0)
        self._telemetery_send_time = time.time()

    def _mission_item_reached_send(self):
        self.mavlink_socket.mav.mission_item_reached_send(self._vehicle.reached_points)
        self._point_seq_send_time = time.time()

    def _send_ack(self, command, status):
        self.mavlink_socket.mav.command_ack_send(command, status)

    def __handler_go_to_local_point(self, msg):
        if len(self.cur_point) != 0:
            if self.cur_point[0] == msg.x and self.cur_point[1] == msg.y:
                self.mavlink_socket.mav.position_target_local_ned_send(0, msg.coordinate_frame,
                                                                       msg.type_mask, msg.x, msg.y, 0, 0, 0,
                                                                       0, 0, 0, 0, 0, 0)
                return
            else:
                self.cur_point = [msg.x, msg.y]
        else:
            self.cur_point = [msg.x, msg.y]
        self.mavlink_socket.mav.position_target_local_ned_send(0,
                                                               msg.coordinate_frame,
                                                               msg.type_mask,
                                                               msg.x,
                                                               msg.y, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        if not self._vehicle.driving_thread.is_alive():
            self._vehicle.driving_thread.start()
        body_fix = False
        if msg.coordinate_frame == mavutil.mavlink.MAV_FRAME_BODY_FRD:
            body_fix = True
        self._vehicle.set_target(self.cur_point[0], self.cur_point[1], body_fix)

    def stop(self):
        self._vehicle.stop()

    def __handler_raspberry_reboot_shutdown(self, msg):  # мб новый поток
        self._vehicle.exit_program()
        time.sleep(1)
        if msg.param1 == 0:
            os.system("sudo shutdown now")
        elif msg.param1 == 1:
            os.system("sudo reboot now")

    def __handler_cmd_do_set_servo(self, msg):
        """
            Обработка команды MAV_CMD_DO_SET_SERVO
        :param msg:
        :return:
        """
        pwm = msg.param1
        servo = msg.param2
        self._vehicle.servo(pwm)

    def __handler_led(self, msg):
        if msg.command == mavutil.mavlink.MAV_CMD_USER_1:  # led control
            try:
                self._led.handler_led_control(msg)
                self._send_ack(msg.command, 0)
            except KeyboardInterrupt:
                print('exit')
                sys.exit()

        elif msg.command == mavutil.mavlink.MAV_CMD_USER_3:  # led custom
            try:
                self._led.handler_led_custom(msg)
                self._send_ack(msg.command, 0)
            except KeyboardInterrupt:
                print('exit')
                sys.exit()


if __name__ == "__main__":
    robot = EdubotVehicle()
    try:
        robot._led._set_color_rgb(255, 255, 255)
        time.sleep(1)
        robot._led._set_color_rgb(0, 0, 0)
        while True:
            pass
    except KeyboardInterrupt:
        sys.exit()
