# -*- coding: utf-8 -*-

import collections

import cv2
import numpy


# Диапазон HSV-цвета препятствий (оранжевые столбы)
OBSTACLE_HSV_LOW = (0, 141, 91)
OBSTACLE_HSV_HIGH = (6, 255, 193)

# Пиксельная ширина контура при минимальном/максимальном расстоянии
_CONTOUR_WIDTH_FAR = 46
_CONTOUR_WIDTH_NEAR = 230

# Ширина кадра (640 px): граница между «влево» и «вправо» — 214 и 480
_FRAME_LEFT_BOUNDARY = 214
_FRAME_RIGHT_BOUNDARY = 480
_FRAME_WIDTH = 640

# Глубина буфера медианного фильтра
_FILTER_HISTORY_LEN = 10


def read_distance(contours, frame):
    """Оценить расстояние до ближайшего препятствия по ширине контура.

    Преобразует ширину bounding-box самого крупного контура в условную
    дистанцию [0, 100].
    """
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    x, y, w, h = cv2.boundingRect(contours[0])
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    distance = (
        (w - _CONTOUR_WIDTH_NEAR)
        * (100 - 0)
        / (_CONTOUR_WIDTH_FAR - _CONTOUR_WIDTH_NEAR)
    )
    return distance


def v_detect():
    """Определить сторону для уклонения и расстояние до препятствия.

    Returns:
        (side, distance): side — 0 влево, 1 вправо; distance — условная [0, 100]
        Если препятствий нет, возвращает (0, 0).
    """
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    try:
        ret, frame = cap.read()
        if not ret:
            return 0, 0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, OBSTACLE_HSV_LOW, OBSTACLE_HSV_HIGH)
        mask = cv2.blur(mask, (4, 4))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        if not contours:
            return 0, 0

        x, y, w, h = cv2.boundingRect(contours[0])

        if x < _FRAME_RIGHT_BOUNDARY:
            side = 1 if x < _FRAME_LEFT_BOUNDARY else 0
        else:
            side = 0

        history = collections.deque(maxlen=_FILTER_HISTORY_LEN)
        distance = read_distance(contours, frame)
        history.append(distance)
        filtered_dist = int(numpy.median(history))

        return side, filtered_dist
    finally:
        cap.release()
