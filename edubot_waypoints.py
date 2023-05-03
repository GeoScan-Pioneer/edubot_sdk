#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

import edubot2
from gs_lps import *
import sys
import math
import time
import threading
import logging


class WaypointsRobot:
    def __init__(self):
        self.edubot = edubot2.EduBot(enableDisplay=False)
        self.edubot.Start()
        logging.basicConfig(level=logging.CRITICAL)
        self._time_start = time.time()
        self._battery_watch_timer = time.time()
        self._nav = us_nav()
        self._nav.start()

        self._telemetery_update_timeout = 0.005
        self._telemetery_update_time = time.time() - self._telemetery_update_timeout
        self._telemetery_alive_timeout = 1
        self._telemetery_get_time = time.time() - self._telemetery_alive_timeout

        self._telemetery_thread = threading.Thread(target=self._update_coordinates)
        self._telemetery_thread.start()

        self.filters = [MedianFilter(), MedianFilter(), MedianFilter()]

        self.x = 0
        self.y = 0
        self.yaw = 0

    def go_to_local_point(self, x_target, y_target, driving):  # доехать до точки с заданными координатами
        if _check_dist_in_circle((self.x, self.y), (x_target, y_target), 0.3):
            self.edubot.Beep()
            logging.info("Point is reached")
            return True

        result_1 = self._rotate_to_target(x_target, y_target, driving)
        result_2 = self._straight_to_target(x_target, y_target, driving)

        if result_2 and result_1:
            self.edubot.Beep()
            logging.info("Point is reached")
            return True
        return False

    def go_to_local_point_body_fixed(self, dx, dy,
                                     driving):  # доехать до точки с заданным смещением относительно текущей позиции
        x, y, yaw = self.x, self.y, self.yaw
        return self.go_to_local_point(x + dx, y + dy, driving)

    def _rotate_to_target(self, x_target, y_target, is_driving, kp=20, ki=0, kd=0, angle_accuracy=0.2):  # повернуться в сторону заданной точки
        e_prev = 0
        e_sum = 0
        x, y, yaw = self.x, self.y, self.yaw
        angle = _normalize_angle(math.atan2(y_target - y, x_target - x) - yaw)
        while abs(angle) > angle_accuracy:
            if not is_driving.is_set():
                self.stop()
                return False
            self._check_battery()
            x, y, yaw = self.x, self.y, self.yaw
            angle = _normalize_angle(math.atan2(y_target - y, x_target - x) - yaw)
            e_sum += angle
            e_sum = _saturate(e_sum, 10)
            u_s = 0
            u_r = _saturate(kp * angle + e_sum * ki + kd * (angle - e_prev), 50)
            self._set_speed(u_s + u_r, u_s - u_r)
            logging.debug(x, y, yaw, u_s, u_r)
            e_prev = angle
        self.stop()
        return True

    def _straight_to_target(self, x_target, y_target, is_driving, kp=32, ki=0, kd=0, dist_accuracy=0.2, speed_max = 80,
                            dist_speedup=0.05, dist_slowdown=0.3):  # движение в точку с динамической коррекцией угла
        e_prev = 0
        e_sum = 0
        x, y, yaw = self.x, self.y, self.yaw
        x_start, y_start = x, y
        dist = math.dist((x, y), (x_target, y_target))
        while dist > dist_accuracy:
            if not is_driving.is_set():
                self.stop()
                return False
            self._check_battery()

            x, y, yaw = self.x, self.y, self.yaw
            dist = math.dist((x, y), (x_target, y_target))
            dist_start = dist = math.dist((x, y), (x_start, y_start))
            angle = _normalize_angle(math.atan2(y_target - y, x_target - x) - yaw)

            if dist_start < dist_speedup:
                u_s = speed_max * (dist_start / dist_speedup)  # разгон на старте
            elif dist < dist_slowdown:
                u_s = speed_max * (dist / dist_slowdown) * 2   # замедление на финише
            else:
                u_s = speed_max                                # движение с максимально возможной линейной скоростью + коррекция угла

            e_sum += angle
            e_sum = _saturate(e_sum, 10)
            u_r = (kp * angle + kd * (angle - e_prev) + e_sum * ki) * _saturate(min(dist, dist_start) * 10, 1)
            self._set_speed(u_s + u_r, u_s - u_r)

            logging.debug(x, y, yaw, u_s, u_r)
            e_prev = angle
            time.sleep(0.025)
        self.stop()
        return True

    def _check_battery(self):
        if time.time() - self._battery_watch_timer >= 5:  # проверка заряда каждые 5 секунд
            voltage = self.edubot.GetPowerData()[0]
            self.battery_watch_timer = time.time()
            if voltage <= 6.6:
                print("low battery")
                self.stop()
                self.edubot.Beep()
                self.exit_program()

    def _set_speed(self, left, right):
        left, right = _saturate(left, 100), _saturate(right, 100)
        self.edubot.rightMotor.SetParrot(round(right))
        self.edubot.leftMotor.SetParrot(round(-left))

    def servo(self, pwm):
        pass

    def _update_coordinates(self):  # получение координат и угла рыскания с локуса
        while True:
            if time.time() - self._telemetery_update_time > self._telemetery_update_timeout:
                if time.time() - self._telemetery_get_time > self._telemetery_alive_timeout:
                    print("Error getting coords from lokus")
                    self._telemetery_get_time = time.time()
                elif time.time() - self._telemetery_update_time > self._telemetery_alive_timeout:
                    print(f'x: {self.x}, y: {self.y}, yaw: {self.yaw}')

                pos = self._nav.get_position()
                angles = self._nav.get_angles()

                if pos is not None and angles is not None and pos[0] != 0 and pos[1] != 0:
                    self._telemetery_update_time = time.time()
                    self._telemetery_get_time = time.time()
                    x, y = pos[0], pos[1]
                    yaw = angles[2] + 3.14  # поворот системы координат
                    x, y, yaw = self.filters[0].filter(x), self.filters[1].filter(y), self.filters[2].filter(yaw)
                    self.x, self.y, self.yaw = x, y, yaw

    def get_local_position(self):
        x, y, yaw = self.x, self.y, self.yaw
        return [x, y, 0]

    def get_attitude(self):
        x, y, yaw = self.x, self.y, self.yaw
        return [0, 0, yaw]

    def stop(self):
        self._set_speed(0, 0)

    def get_battery_status(self):
        voltage, current, power = self.edubot.GetPowerData()
        return [voltage, current, power]

    def exit_program(self):
        print("exit")
        self.stop()
        self.edubot.Release()

class MedianFilter:  # медианный фильтр для фильтрации одиночных выбросов
    history = [-10]

    def filter(self, x):
        self.history.append(x)
        while len(self.history) > 3:
            self.history.pop(0)
        return sorted(self.history)[1]


def _saturate(value, limit):  # ограничение значения отрезком [-limit; limit]
    limit = abs(limit)
    if value < -limit:
        return -limit
    elif value > limit:
        return limit
    else:
        return value


def _normalize_angle(angle):
    while angle < -math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle


def _check_dist_in_circle(p1, p2, dist):
    d = math.dist(p1, p2)
    return True if d < dist else False


if __name__ == "__main__":
    robot = WaypointsRobot()
    is_driving = threading.Event()
    is_driving.set()
    if sys.argv[1] and sys.argv[2]:
        x_target, y_target = float(sys.argv[1]), float(sys.argv[2])
        robot.go_to_local_point(x_target, y_target, is_driving)
