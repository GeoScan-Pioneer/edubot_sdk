#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
import dataclasses
import logging
import edubot2
from gs_lps import *
import sys
import math
import time
import threading

# коэффициенты для edubot
KP = 25  # ref 32
KI = 0
KD = 0  # ref 2.5
KP_2 = 20  # ref 20
KI_2 = 0  # ref 2
KD_2 = 0
DIST_ACCURACY = 0.2  # ref 0.05
ANGLE_ACCURACY = 0.1  # ref: 0.03
SPEED_MAX = 80
INT_SUM_LIMIT = 10

dist_speedup = 0.1
dist_slowdown = 0.3


@dataclasses.dataclass
class Point:
    """
    Класс для описания точек
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class WaypointsRobot:
    def __init__(self):
        self.edubot = edubot2.EduBot(enableDisplay=False)
        self.edubot.Start()
        logging.basicConfig(level=logging.DEBUG)
        self._time_start = time.time()
        self._battery_watch_timer = time.time()
        self._nav = us_nav()
        self._nav.start()

        self._telemetery_update_timeout = 0.025
        self._telemetery_update_time = time.time() - self._telemetery_update_timeout
        self._telemetery_alive_timeout = 1
        self._telemetery_get_time = time.time() - self._telemetery_alive_timeout

        self._x_filter = MedianFilter()
        self._y_filter = MedianFilter()
        self._yaw_filter = MedianFilter()

        self.cur_coord = Point()

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
        if body_fixed:
            self.cur_target = Point(self.cur_coord.x + x, self.cur_coord.y + y, 0)
        else:
            self.cur_target = Point(x, y, 0)

    def go_to_local_point(self):
        """Движение в точку с динамической коррекцией угла
        """

        # print(f'target: {x_target}, {y_target}')
        int_sum = 0
        e_prev = 0
        point_to_go = Point()
        point_start = Point()
        while True:
            time.sleep(0.05)
            if self.cur_target is None:
                continue
            elif self.cur_target.x != point_to_go.x or self.cur_target.y != point_to_go.y:  # обнаружили, что цель изменилась
                print("new point")
                self.new_point.set()
                self.point_reached.clear()
                point_to_go = self.cur_target

            dist = self.dist_to(point_to_go)
            print(dist)

            if self.point_reached.is_set():
                self.stop()
                continue
            elif dist <= DIST_ACCURACY:
                self.stop()
                self.point_reached.set()  # (нужно чтобы зайти в это условие 1 раз)
                self.reached_points += 1
                print("point_reached")
                continue

            if self.new_point.is_set():  # если обнаружена новая точка, повернуть
                self.new_point.clear()
                self.stop()
                self.rotate_to(point_to_go)  # rotate должен быть позже проверки на point_reached
                continue
            angle = self.angle_to(point_to_go)
            dist_start = self.dist_to(point_start)
            if dist_start < dist_speedup:
                u_s = SPEED_MAX * (dist_start / dist_speedup)  # разгон на старте
            elif dist < dist_slowdown:
                u_s = SPEED_MAX * (dist / dist_slowdown)  # замедление на финише
            else:
                u_s = SPEED_MAX  # движение с максимально возможной линейной скоростью + коррекция угла
            e_sum = saturate(int_sum + angle, 10)
            u_r = (KP * angle + KD * (angle - e_prev) + e_sum * KI)
            e_prev = angle
            self._set_speed(u_s + u_r, u_s - u_r)
            logging.debug(self.cur_coord, angle, u_s, u_r)
        return True

    def rotate_to(self, point_to_go: Point):
        e_prev = 0
        int_sum = 0
        angle = ANGLE_ACCURACY + 1
        while abs(angle) > ANGLE_ACCURACY:
            angle = self.angle_to(point_to_go)
            int_sum = saturate(int_sum + angle, INT_SUM_LIMIT)
            u_r = saturate(KP_2 * angle + KD_2 * (angle - e_prev) + KI_2 * int_sum, 50)
            self._set_speed(u_r, - u_r)
            print(self.cur_coord, angle, u_r)
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
                    yaw = angles[2] - math.pi  # поворот системы координат
                    x, y, yaw = self._x_filter.filter(x), self._y_filter.filter(y), self._yaw_filter.filter(yaw)
                    self.cur_coord = Point(x, y, yaw)

                if time.time() - self._telemetery_get_time > self._telemetery_alive_timeout:
                    print("Одиночная ошибка получения координат с локуса")
                    self._telemetery_get_time = time.time()
                elif time.time() - self._telemetery_update_time > self._telemetery_alive_timeout:
                    # print(f'x: {self.x}, y: {self.y}, yaw: {self.yaw}')
                    pass

    def get_local_position(self):
        return [self.cur_coord.x, self.cur_coord.y, 0]

    def get_attitude(self):
        return [0, 0, self.cur_coord.yaw]

    def _set_speed(self, left, right):
        left = round(saturate(left, 100))
        right = round(saturate(right, 100))
        self.edubot.rightMotor.SetParrot(right)
        self.edubot.leftMotor.SetParrot(-left)

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

    def dist_to(self, point: Point):
        return math.sqrt((self.cur_coord.x - point.x) ** 2 + (self.cur_coord.y - point.y) ** 2)

    def angle_to(self, point: Point):
        return normalize_angle(math.atan2(point.y - self.cur_coord.y, point.x - self.cur_coord.x) - self.cur_coord.yaw)


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

        line = [Point(-0, 0),
                Point(3, 3),
                Point(-4.2, 0.4)]

        for p in line:
            robot.set_target(p.x, p.y)
            robot.go_to_local_point()
            while not robot.point_reached:
                pass
            robot.beep()

    except KeyboardInterrupt:
        pass

    robot.exit_program()
