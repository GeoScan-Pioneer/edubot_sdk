#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
import logging

import edubot2
from gs_lps import *
import sys
import math
import time
import threading

KR = 35
KD = 4
KP = 32
LIMIT = 0.05
SPEED_MAX = 90
dist_speedup = 0.3
dist_max = 1


class WaypointsRobot:
    def __init__(self):
        self.edubot = edubot2.EduBot(enableDisplay=False)
        self.edubot.Start()

        self._time_start = time.time()
        self._battery_watch_timer = time.time()

        self._telemetery_thread = threading.Thread(target=self._update_coordinates, daemon=True)
        self._telemetery_thread.daemon = True
        self._telemetery_thread.start()
        self._telemetery_timeout = 0.025
        self._telemetery_send_time = time.time() - self._telemetery_timeout

        self._x_filter = MedianFilter()
        self._y_filter = MedianFilter()
        self._yaw_filter = MedianFilter()

        self.x = 0
        self.y = 0
        self.yaw = 0

    def go_to_local_point(self, x_target, y_target, driving):  # доехать до точки с заданными координатами
        result_1 = self._rotate_to_target(x_target, y_target, driving)
        result_2 = self._straight_to_target(x_target, y_target, driving)
        if result_2 and result_1:
            return True
        return False

    def go_to_local_point_body_fixed(self, dx, dy,
                                     driving):  # доехать до точки с заданным смещением относительно текущей позиции
        x, y, yaw = self.x, self.y, self.yaw
        return self.go_to_local_point(x + dx, y + dy, driving)

    def _rotate_to_target(self, x_target, y_target, is_driving):  # повернуться в сторону заданной точки
        e_prev = 0
        e_sum = 0
        x, y, yaw = self.x, self.y, self.yaw
        angle = normalize_angle(math.atan2(y_target - y, x_target - x) - yaw)
        while abs(angle) > 0.03:
            if not is_driving.is_set():
                self.stop()
                return False
            self._check_battery()
            x, y, yaw = self.x, self.y, self.yaw
            angle = normalize_angle(math.atan2(y_target - y, x_target - x) - yaw)
            e_sum += angle
            u_s = 0
            u_r = KR * angle + KD * (angle - e_prev)
            self._set_speed(u_s + u_r, u_s - u_r)
            logging.debug(x, y, yaw, u_s, u_r)
            e_prev = angle
            time.sleep(0.025)
        self.stop()
        return True

    def _straight_to_target(self, x_target, y_target, driving):  # движение в точку с динамической коррекцией угла
        e_prev = 0
        x, y, yaw = self.x, self.y, self.yaw
        x_start, y_start = x, y
        dist = math.sqrt((x_target - x) ** 2 + (y_target - y) ** 2)
        while dist > LIMIT:
            if not driving.is_set():
                self.stop()
                return False
            self._check_battery()
            x, y, yaw = self.x, self.y, self.yaw
            dist = math.sqrt((x_target - x) ** 2 + (y_target - y) ** 2)
            dist_start = math.sqrt((x_start - x) ** 2 + (y_start - y) ** 2)
            angle = normalize_angle(math.atan2(y_target - y, x_target - x) - yaw)
            # if dist_start < dist_speedup:
            #     u_s = SPEED_MAX * math.sqrt(dist_start / dist_speedup)  # разгон на старте
            # elif dist_speedup > dist:
            #     u_s = SPEED_MAX * math.sqrt(dist / dist_speedup)  # замедление на финише
            # else:
            #     u_s = SPEED_MAX  # движение с максимально возможной линейной скоростью + коррекция угла
            u_s = SPEED_MAX * saturate(math.sqrt(min(dist_start, dist) / dist_speedup), 1)
            u_r = KP * angle + KD * (angle - e_prev)
            e_prev = angle
            self._set_speed(u_s + u_r, u_s - u_r)
            logging.debug(x, y, yaw, u_s, u_r)
            time.sleep(0.025)
        self.stop()
        return True

    def _set_speed(self, left, right):
        left, right = saturate(left, 100), saturate(right, 100)
        self.edubot.rightMotor.SetParrot(round(right))
        self.edubot.leftMotor.SetParrot(round(-left))

    def _update_coordinates(self):  # получение координат и угла рыскания с локуса
        self._nav = us_nav()
        self._nav.start()
        while True:
            pos = self._nav.get_position()
            angles = self._nav.get_angles()
            if pos is not None and angles is not None:  # при сбое могут прийти координаты (0, 0)
                x, y = self._x_filter.filter(pos[0]), self._y_filter.filter(pos[1])
                yaw = self._yaw_filter.filter(
                    angles[2]) + 3.14  # поворот системы координат, чтобы угол отсчитывался от оси x
                if time.time() - self._telemetery_send_time >= self._telemetery_timeout:
                    self._telemetery_send_time = time.time()
                    self.x, self.y, self.yaw = x, y, yaw

    def get_local_position(self):
        return [self.x, self.y, 0]

    def get_attitude(self):
        return [0, 0, self.yaw]

    def _check_battery(self):
        if time.time() - self._battery_watch_timer >= 5:  # проверка заряда каждые 5 секунд
            voltage = self.edubot.GetPowerData()[0]
            self.battery_watch_timer = time.time()
            if voltage <= 6.6:
                print("low battery")
                self.stop()
                self.edubot.Beep()
                self.exit_program()

    def stop(self):
        self._set_speed(0, 0)

    def get_battery_status(self):
        return self.edubot.GetPowerData()

    def exit_program(self):
        print("exit")
        self.stop()
        self.edubot.Release()
        sys.exit()


class MedianFilter:  # медианный фильтр для фильтрации одиночных выбросов
    history = [-10]

    def filter(self, x):
        self.history.append(x)
        while len(self.history) > 3:
            self.history.pop(0)
        return sorted(self.history)[1]


def saturate(value, limit):  # ограничение значения отрезком [-limit; limit]
    limit = abs(limit)
    if value < -limit:
        return -limit
    elif value > limit:
        return limit
    else:
        return value


def normalize_angle(angle):
    while angle < -math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle
