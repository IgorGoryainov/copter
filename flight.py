# -*- coding: utf-8 -*-

import math
import time

import rospy
import RPi.GPIO as GPIO
from clever import srv
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger

import planner

# ─── Константы ───────────────────────────────────────────────────────────────

TAKEOFF_HEIGHT = 1.5       # высота взлёта, м
TAKEOFF_SPEED = 0.5        # скорость взлёта, м/с
WAYPOINT_TOLERANCE = 0.8   # радиус «зачёта» точки, м
OBSTACLE_DISTANCE_THRESHOLD = 60  # порог срабатывания уклонения, у.е.
ELECTROMAGNET_PIN = 18     # GPIO BCM pin для управления электромагнитом
POINTS_FILE = 'points.txt'

# ─── Инициализация ────────────────────────────────────────────────────────────

rospy.init_node('flight')

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setup(ELECTROMAGNET_PIN, GPIO.OUT)

arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
land = rospy.ServiceProxy('land', Trigger)

rate = rospy.Rate(10)

# ─── Загрузка маршрута ────────────────────────────────────────────────────────

waypoints = planner.read_waypoints(POINTS_FILE)
total_waypoints = len(waypoints)
current_waypoint = 0

# ─── Взлёт ───────────────────────────────────────────────────────────────────

navigate(x=0, y=0, z=TAKEOFF_HEIGHT, speed=TAKEOFF_SPEED, frame_id='body', auto_arm=True)
time.sleep(5)

GPIO.output(ELECTROMAGNET_PIN, GPIO.HIGH)  # удерживаем груз

# ─── Основной цикл ────────────────────────────────────────────────────────────

while not rospy.is_shutdown():
    target_x, target_y, target_z, drop_cargo = map(float, waypoints[current_waypoint].split())
    drop_cargo = int(drop_cargo)

    telemetry = get_telemetry(frame_id='aruco_map')
    pos_x, pos_y, pos_z = telemetry.x, telemetry.y, telemetry.z

    # Уклонение от препятствий
    side, distance = planner.check_env()
    if distance < OBSTACLE_DISTANCE_THRESHOLD:
        if side == 0:    # влево
            set_position(x=0, y=1, z=0, frame_id='body')
            time.sleep(2)
        elif side == 1:  # вправо
            set_position(x=0, y=-1, z=0, frame_id='body')
            time.sleep(2)
        elif side == 2:  # назад
            set_position(x=-1, y=0, z=0, frame_id='body')
            time.sleep(2)

    # Доворот к целевой точке и движение вперёд
    bearing = planner.nav4(pos_x, pos_y, target_x, target_y)
    set_position(
        x=telemetry.x, y=telemetry.y, z=TAKEOFF_HEIGHT,
        yaw=math.radians(bearing), frame_id='aruco_map',
    )
    set_position(x=1, y=0, z=telemetry.z, yaw=float('nan'), frame_id='body')

    # Проверяем, достигнута ли точка
    dist_to_target = math.sqrt(
        (target_x - pos_x) ** 2
        + (target_y - pos_y) ** 2
        + (target_z - TAKEOFF_HEIGHT) ** 2
    )
    if dist_to_target < WAYPOINT_TOLERANCE:
        current_waypoint += 1

        if drop_cargo == 1:
            # Садимся, сбрасываем груз, взлетаем снова
            set_position(x=0, y=0, z=-2, frame_id='body')
            time.sleep(4)
            arming(False)
            GPIO.output(ELECTROMAGNET_PIN, GPIO.LOW)
            time.sleep(5)
            navigate(x=0, y=0, z=TAKEOFF_HEIGHT, speed=TAKEOFF_SPEED, frame_id='body', auto_arm=True)
            time.sleep(5)

    if current_waypoint >= total_waypoints:
        print('All waypoints complete — landing')
        break

    rate.sleep()

# ─── Посадка ──────────────────────────────────────────────────────────────────

set_position(x=0, y=0, z=-2, frame_id='body')
time.sleep(4)

final_telemetry = get_telemetry(frame_id='aruco_map')
print('Landed at:', final_telemetry.x, final_telemetry.y)
arming(False)
