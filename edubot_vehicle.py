import os
import sys
from pymavlink.dialects.v20.common import *
from pymavlink import mavutil
import time
import threading
import socket
import edubot_waypoints2
import logging
import led


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

    _SUPPORTED_CONNECTION_METHODS = ['serial', 'udpin', "udpout"]

    def __init__(self, name='EdubotVehicle', ip=None, mavlink_port=5656, connection_method='udpin',
                 device='/dev/serial0', baud=115200, logging_level='INFO'):

        self.name = name
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


        self._vehicle = edubot_waypoints2.WaypointsRobot()
        self._led = led.Led()

        self._is_connected = False
        self._is_connected_timeout = 1
        self._last_msg_time = time.time() - self._is_connected_timeout

        self._heartbeat_timeout = 1
        self._heartbeat_send_time = time.time() - self._heartbeat_timeout

        self._telemetery_timeout = 1
        self._telemetery_send_time = time.time() - self._telemetery_timeout

        self._point_seq = 0
        self._point_seq_timeout = 1
        self._point_seq_send_time = time.time() - self._point_seq_timeout

        self._is_driving = threading.Event()
        self._driving_thread = None

        self._mavlink_send_timeout = 0.5
        self._mavlink_send_long_timeout = 1
        self._mavlink_send_number = 10

        self.mavlink_socket = self._create_connection(connection_method=connection_method,
                                                      ip=ip, port=mavlink_port,
                                                      device=device, baud=baud)
        self.__is_socket_open = threading.Event()
        self.__is_socket_open.set()

        self.msg_archive = dict()
        self.wait_msg = dict()

        self.command_long_handler = {
            MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: lambda msg: self._raspberry_reboot_shutdown(msg),
            MAV_CMD_DO_SET_SERVO: lambda msg: self.__handler_cmd_do_set_servo(msg),
            MAV_CMD_USER_1: lambda msg: self.__handler_led(msg),
            MAV_CMD_USER_3: lambda msg: self.__handler_led(msg)
        }

        self._message_handler_thread = threading.Thread(target=self._message_handler, daemon=True)
        self._message_handler_thread.daemon = True
        self._message_handler_thread.start()

        logging.info(f"[{self.name}] <Connection> connecting to station...")
        # mavwifi.Wifi.__init__(self, self.mavlink_socket)

    def __del__(self):
        logging.debug(f"[{self.name}] <Object> Class object removed")

    def _create_connection(self, connection_method, ip, port, device, baud):
        """
        create mavlink connection
        :return: mav_socket
        """
        if connection_method not in self._SUPPORTED_CONNECTION_METHODS:
            logging.error(f"[{self.name}] <Connection> Unknown connection method: {connection_method}")

        mav_socket = None
        try:
            if connection_method == "serial":
                mav_socket = mavutil.mavlink_connection(device=device, baud=baud)
            else:
                mav_socket = mavutil.mavlink_connection('%s:%s:%s' % (connection_method, ip, port))
                mav_socket.mav.mission_item_reached_send(0)

            return mav_socket

        except socket.error as e:
            logging.error(f"[{self.name}] <Connection> Connection error. Can not connect to station: {e}")

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

            if time.time() - self._heartbeat_send_time >= self._heartbeat_timeout:
                self._send_heartbeat()
            if time.time() - self._telemetery_send_time >= self._telemetery_timeout:
                self._attitude_send()
                self._local_position_ned_send()
            if time.time() - self._point_seq_send_time >= self._point_seq_timeout:
                self._mission_item_reached_send()

            msg = self.mavlink_socket.recv_msg()
            if msg is not None:
                print(f"[{self.name}] <Message> {msg}")

                if msg.id == MAVLINK_MSG_ID_COMMAND_LONG:
                    self._send_ack(msg.command, 0)
                    self.command_long_handler[msg.command](msg)

                self._last_msg_time = time.time()
                if not self._is_connected:
                    self._is_connected = True
                    logging.info(f"[{self.name}] <Connection> connected to station")

                if msg.id == MAVLINK_MSG_ID_HEARTBEAT:
                    pass
                elif msg.id == MAVLINK_MSG_ID_COMMAND_ACK:
                    msg._type += f'_{msg.command}'

                elif msg.id == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:  # команда ехать в точку
                    self._driving_thread = threading.Thread(target=self._go_to_local_point, args=[msg,
                                                                                                  self._is_driving])  # отдать задачу в отдельный thread
                    self._driving_thread.start()

                if msg.get_type() in self.wait_msg:  # если находится в листе ожидания (например, ack)
                    self.wait_msg[msg.get_type()].set()  # триггерим Event

                self.msg_archive.update({msg.get_type(): {'msg': msg, 'is_read': threading.Event()}})

            elif self._is_connected and (time.time() - self._last_msg_time > self._is_connected_timeout):
                self._is_connected = False  # если
                logging.info(f"[{self.name}] <Connection> disconnected")

        logging.info(f"[{self.name}] <Object> message handler stopped")

    def _sys_status_send(self):  # msgname = "SYS_STATUS"
        self.mavlink_socket.mav.sys_status_send()

    def _battery_status_send(self):
        voltages = self._vehicle.get_battery_status()
        self.mavlink_socket.mav.battery_status_send(0, 0, 0, 0, voltages, 0, 0, 0, 0)

    def _attitude_send(self):
        att = self._vehicle.get_attitude()
        self.mavlink_socket.mav.attitude_send(0, att[0], att[1], att[2], 0, 0, 0)
        self._telemetery_send_time = time.time()

    def _local_position_ned_send(self):
        pos = self._vehicle.get_local_position()
        self.mavlink_socket.mav.local_position_ned_send(0, pos[0], pos[1], pos[2], 0, 0, 0)
        self._telemetery_send_time = time.time()

    def _mission_item_reached_send(self):
        self.mavlink_socket.mav.mission_item_reached_send(self._point_seq)
        self._point_seq_send_time = time.time()

    def _send_ack(self, command, status):
        self.mavlink_socket.mav.command_ack_send(command, status)

    def _go_to_local_point(self, msg, event):
        # time_boot_ms, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
        self.mavlink_socket.mav.position_target_local_ned_send(0, msg.coordinate_frame,
                                                               msg.type_mask, msg.x, msg.y, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        if self._is_driving.is_set():
            self._is_driving.clear()
            time.sleep(0.5)
        self._is_driving.set()
        if msg.coordinate_frame == mavutil.mavlink.MAV_FRAME_LOCAL_ENU:
            if self._vehicle.go_to_local_point(msg.x, msg.y, event):
                self._point_seq += 1
        elif msg.coordinate_frame == mavutil.mavlink.MAV_FRAME_BODY_FRD:
            if self._vehicle.go_to_local_point_body_fixed(msg.x, msg.y, event):
                self._point_seq += 1

    def stop(self):
        self._vehicle.stop()

    def _raspberry_reboot_shutdown(self, msg):  # мб новый поток
        self._vehicle.exit_program()
        time.sleep(2)
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
        print(f'PWM {pwm}')
        print(f'Servo {servo}')
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
        while True:
            pass
    except KeyboardInterrupt:
        sys.exit()
