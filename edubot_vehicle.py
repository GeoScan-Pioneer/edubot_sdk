import os

from pymavlink import mavutil
# from .mavsub import wifi as mavwifi
import time
import threading
import socket
import edubot_waypoints_test

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

    _SUPPORTED_CONNECTION_METHODS = ['serial', 'udpin']
    def __init__(self, name='edubotVehicle', ip='localhost', mavlink_port=8001, connection_method='serial',
                 device='/dev/serial0', baud=115200,
                 logger=True, log_connection=True):

        self.name = name

        self._vehicle = edubot_waypoints_test.WaypointsRobot(logger)

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

        self._logger = logger
        self._log_connection = log_connection


        self.mavlink_socket = self._create_connection(connection_method=connection_method,
                                                      ip=ip, port=mavlink_port,
                                                      device=device, baud=baud)
        self.__is_socket_open = threading.Event()
        self.__is_socket_open.set()

        self.msg_archive = dict()
        self.wait_msg = dict()

        self._message_handler_thread = threading.Thread(target=self._message_handler, daemon=True)
        self._message_handler_thread.daemon = True
        self._message_handler_thread.start()


        self.log(msg_type='connection', msg='Connecting to station...\n')
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
        """
        Вывести сообщение
        :return: None
        """
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
            if time.time() - self._telemetery_send_time >= self._telemetery_timeout:
                self._attitude_send()
                self._local_position_ned_send()
                self._battery_status_send()
            if time.time() - self._point_seq_send_time >= self._point_seq_timeout:
                self._mission_item_reached_send()

            msg = self.mavlink_socket.recv_msg()
            if msg is not None:
                self.log(msg)
                self._last_msg_time = time.time()
                if not self._is_connected:
                    self._is_connected = True
                    self.log(msg_type='connection', msg='CONNECTED')
                if msg.get_type() == 'HEARTBEAT':
                    pass
                elif msg.get_type() == 'COMMAND_ACK':
                    msg._type += f'_{msg.command}'

                elif msg.get_type() == 'SET_POSITION_TARGET_LOCAL_NED':                    # команда ехать в точку

                    if msg.coordinate_frame == mavutil.mavlink.MAV_FRAME_LOCAL_ENU: # в абсолютных координатах
                        self._driving_thread = threading.Thread(target=self._go_to_local_point, args= [msg, self._is_driving])
                        self._driving_thread.start()
                    elif msg.coordinate_frame == mavutil.mavlink.MAV_FRAME_BODY_FRD: # в относительных старта координатах
                        self._driving_thread = threading.Thread(target=self._go_to_local_point_body_fixed, args= [msg, self._is_driving])
                        self._driving_thread.start()

                if msg.get_type() in self.wait_msg:      # если находится в листе ожидания (например, ждем ack)
                    self.wait_msg[msg.get_type()].set()  # триггерим Event - дождались

                self.msg_archive.update({msg.get_type(): {'msg': msg, 'is_read': threading.Event()}})

            elif self._is_connected and (time.time() - self._last_msg_time > self._is_connected_timeout):
                self._is_connected = False # если
                self.log(msg_type='connection', msg='DISCONNECTED')

        self.log(msg="Message handler stopped")

    def _sys_status_send(self): # msgname = "SYS_STATUS"
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


    def _send_ack(self, msg, status):
        self.mavlink_socket.mav.command_ack_send(msg, status)


    def _go_to_local_point(self, msg, event):
        # time_boot_ms, coordinate_frame, type_mask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
        self.mavlink_socket.mav.position_target_local_ned_send(0, msg.coordinate_frame,
                                                               msg.type_mask, msg.x, msg.y, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        if self._is_driving.is_set():
            self._is_driving.clear()
            time.sleep(0.5)  # todo: после теста на реальном роботе изменить
        self._is_driving.set()
        if self._vehicle.go_to_local_point(msg.x, msg.y, event):
            self._point_seq += 1



    def _go_to_local_point_body_fixed(self, msg, event):
        self.mavlink_socket.mav.position_target_local_ned_send(0, msg.target_system, msg.target_component,
                                                               msg.coordinate_frame,
                                                               msg.type_mask, msg.x, msg.y, 0, 0, 0, 0, 0, 0, 0)
        if self._is_driving.is_set():
            self._is_driving.clear()
            time.sleep(0.2)
        self._is_driving.set()
        if self._vehicle.go_to_local_point_body_fixed(msg.x, msg.y, event):
            self._point_seq += 1


    def stop(self):
        self._vehicle.stop()


    def _raspberry_poweroff(self, msg): #мб новый поток
        self._send_ack(msg.command, 2)
        self._vehicle.exit_program()
        os.system("sudo shutdown now")

    def _raspberry_reboot(self, msg):
        self._send_ack(msg.command, 2)
        self._vehicle.exit_program()
        os.system("sudo reboot now")

