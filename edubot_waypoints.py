#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
import logging

import edubot2
from gs_lps import *
import sys
import math
import time
import threading

# коэффициенты для edubot
DIST_ACCURACY = 0.25
ANGLE_ACCURACY = 0.2
SPEED_MAX = 230
dist_speedup = 0.1
INT_SUM_LIMIT = 10


class WaypointsRobot:
    def __init__(self):
        self.edubot = edubot2.EduBot(enableDisplay=False)
        self.edubot.Start()
        logging.basicConfig(level=logging.DEBUG)
        self._time_start = time.time()
        self._battery_watch_timer = time.time()
        self._nav = us_nav()
        self._nav.start()

        self._telemetery_update_timeout = 0.05
        self._telemetery_update_time = time.time() - self._telemetery_update_timeout
        self._telemetery_alive_timeout = 1
        self._telemetery_get_time = time.time() - self._telemetery_alive_timeout

        self._x_filter = MedianFilter()
        self._y_filter = MedianFilter()
        self._yaw_filter = MedianFilter()

        self.x = 0
        self.y = 0
        self.yaw = 0

        self._telemetery_thread = threading.Thread(target=self._update_coordinates)
        self._telemetery_thread.start()

        self.driving_thread = threading.Thread(target=self.go_to_local_point)

        self.cur_target = None
        self.new_point = threading.Event()
        self.new_point.clear()
        self.point_reached = threading.Event()
        self.point_reached.clear()
        self.reached_points = 0

    def set_target(self, x, y, body_fixed=False):
        self.cur_target = [x, y]
        if body_fixed is True:
            self.cur_target = [self.x+x, self.y+y]


    def go_to_local_point(self, KP=200, KD=1, KI=15):
        """Движение в точку с динамической коррекцией угла

        Args:
            x_target (_type_): _description_
            y_target (_type_): _description_
            KP (int, optional): _description_. Defaults to 2000.
            KD (int, optional): _description_. Defaults to 10.
            KI (int, optional): _description_. Defaults to 25.
        """

        # print(f'target: {x_target}, {y_target}')
        int_sum = 0
        e_prev = 0
        x_target = 0
        y_target = 0

        while True:
            if self.cur_target is None:
                continue
            if self.cur_target[0] != x_target or self.cur_target[1] != y_target: # если обнаружили, что точка изменилась
                self.new_point.set()
                self.point_reached.clear()
                x_target = self.cur_target[0] # обновить координаты
                y_target = self.cur_target[1]
            x, y, yaw = self.x, self.y, self.yaw
            dist = math.sqrt((x_target - x) ** 2 + (y_target - y) ** 2)
            if dist <= DIST_ACCURACY and not self.new_point.is_set() and not self.point_reached.is_set(): # если мы уже в точке и она не отмечена достигнутой и не была задана новая
                self.stop()
                self.point_reached.set() # отметить, что достигли последнюю (нужно чтобы зайти в это условие 1 раз)
                self.reached_points += 1
                continue
            elif dist <= DIST_ACCURACY:
                continue
            if self.new_point.is_set():    # если точка сменилась
                self.point_reached.clear() # новую точку еще не достигли
                self.new_point.clear()     # новую точку прочитали и она уже не новая
                self.stop()                # остановить движение
                self.rotate_to_target2(x_target, y_target)   # довернуться до новой точки
                continue
            angle = normalize_angle(math.atan2(
                    y_target - y, x_target - x) - yaw)
            int_sum += angle
            int_sum = saturate(int_sum, INT_SUM_LIMIT)

            u_s = SPEED_MAX
            u_r = (KP * angle + KD * (angle - e_prev) + KI * int_sum)
            u_r_min = 80
            if u_r > u_r_min:
                self._set_speed(u_s, u_s - u_r)
            elif u_r < u_r_min:
                self._set_speed(u_s + u_r, u_s)
            else:
                self._set_speed(SPEED_MAX, SPEED_MAX)
            # self.set_speed(u_s + u_r, u_s - u_r)
            e_prev = angle
            # print(x, y, angle, u_s, u_r)
        self._set_speed(0, 0)
        return True

    def rotate_to_target2(self, x_target, y_target, KP=150, KD=1, KI=17):
        e_prev = 0
        int_sum = 0
        angle = ANGLE_ACCURACY + 1
        while abs(angle) > ANGLE_ACCURACY:
            x, y, yaw = self.x, self.y, self.yaw
            angle = normalize_angle(math.atan2(
                    y_target - y, x_target - x) - yaw)

            int_sum += angle
            int_sum = saturate(int_sum, INT_SUM_LIMIT)
            u_r = KP * angle + KD * (angle - e_prev) + KI * int_sum
            self._set_speed(u_r, - u_r)
            print(x, y, angle, u_r)
            e_prev = angle
        self._set_speed(0, 0)
        return True

    def _update_coordinates(self):  # получение координат и угла рыскания с локуса
        while True:
            if time.time() - self._telemetery_update_time > self._telemetery_update_timeout:

                pos = self._nav.get_position()
                angles = self._nav.get_angles()

                if pos is not None and angles is not None and pos[0] != 0 and pos[1] != 0:
                    self._telemetery_update_time = time.time()
                    self._telemetery_get_time = time.time()
                    x, y = pos[0], pos[1]
                    yaw = angles[2]# поворот системы координат
                    # x, y, yaw = self._x_filter.filter(x), self._y_filter.filter(y), self._yaw_filter.filter(yaw)
                    self.x, self.y, self.yaw = x, y, yaw
                    # print(f'x: {self.x}, y: {self.y}, yaw: {self.yaw}')

                if time.time() - self._telemetery_get_time > self._telemetery_alive_timeout:
                    print("Одиночная ошибка получения координат с локуса")
                    self._telemetery_get_time = time.time()
                elif time.time() - self._telemetery_update_time > self._telemetery_alive_timeout:
                    #print(f'x: {self.x}, y: {self.y}, yaw: {self.yaw}')
                    pass


    def get_local_position(self):
        x, y, yaw = self.x, self.y, self.yaw
        return [x, y, 0]

    def get_attitude(self):
        x, y, yaw = self.x, self.y, self.yaw
        return [0, 0, yaw]

    def _set_speed(self, left, right):
        left = round(saturate(left, 240))
        right = round(saturate(right, 240))
        # print(left, right)
        self.edubot.rightMotor.SetSpeed(right)
        self.edubot.leftMotor.SetSpeed(left)


    def _check_battery(self):
        if time.time() - self._battery_watch_timer >= 5:  # проверка заряда каждые 5 секунд
            voltage = self.edubot.GetPowerData()[0]
            self.battery_watch_timer = time.time()
            if voltage <= 5:
                print("low battery")
                # self.stop()
                # self.edubot.Beep()
                # self.exit_program()

    def stop(self):
        self._set_speed(0, 0)

    def get_battery_status(self):
        voltage, current, power = self.edubot.GetPowerData()
        return [voltage, current, power]

    def exit_program(self):
        print("exit")
        self.stop()
        self.edubot.Release()

    def beep(self):
        self.edubot.Beep()

    def servo(self, pwm):
        self.edubot.servo(pwm)


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


if __name__ == "__main__":
    try:
        is_driving = threading.Event()
        is_driving.set()
        robot = WaypointsRobot()

        line = [(-0, 0),
                (3, 3),
                (-4.2, 0.4)]

        for dot in line:
            robot.go_to_local_point(dot[0], dot[1], is_driving)
            robot.beep()

    except KeyboardInterrupt:
        pass

    robot.exit_program()