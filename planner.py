# -*- coding: utf-8 -*-

import math


def nav4(cur_x, cur_y, target_x, target_y):
    """Вычислить угол поворота (в градусах) в системе координат aruco_map.

    Возвращает угол в диапазоне [-180, 180], необходимый для разворота коптера
    в направлении целевой точки.
    """
    dx = target_x - cur_x
    dy = target_y - cur_y
    hyp = math.sqrt(dx ** 2 + dy ** 2)
    sina = dy / hyp
    cosa = dx / hyp

    if sina >= 0 and cosa >= 0:
        angle = math.degrees(math.acos(cosa))
    elif sina >= 0 and cosa < 0:
        angle = 180 - math.degrees(math.acos(abs(cosa)))
    elif sina < 0 and cosa < 0:
        angle = -(180 - math.degrees(math.acos(abs(cosa))))
    else:
        angle = -math.degrees(math.acos(cosa))

    # Cardinal directions
    if sina == 0 and cosa == 1:
        angle = 0
    elif sina == 1 and cosa == 0:
        angle = 90
    elif sina == 0 and cosa == -1:
        angle = 180
    elif sina == -1 and cosa == 0:
        angle = -90

    return angle


def read_waypoints(filepath):
    """Прочитать точки маршрута из файла.

    Каждая строка формата: x y z drop_cargo
    Возвращает список строк.
    """
    with open(filepath, 'r') as f:
        return [line.strip() for line in f if line.strip()]


def check_env():
    """Вернуть направление уклонения и расстояние до препятствия.

    Returns:
        (side, distance): side — 0 влево, 1 вправо, 2 назад
    """
    from video_detect import v_detect
    side, dist = v_detect()
    return int(side), int(dist)
