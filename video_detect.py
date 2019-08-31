# -*- coding: utf-8 -*-

import cv2
import collections
import numpy

#перед выводом данных, используем медианный фильтр

def read_distance(contours, frame):
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    x, y, w, h = cv2.boundingRect(contours[0])
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    OldRange = (46 - 230)
    NewRange = (100 - 0)
    NewValue = (((w - 230) * NewRange) / OldRange) # этих трех строках происходит перевод из одной системы отсчета в другую, для определения приблизительного расстояния до препятствия
    return (NewValue)

def read_distance_filtered(contours, frame, history):
    history.append(read_distance(contours, frame))
    return numpy.median(history)

def v_detect():
    #cap = cv2.VideoCapture(0) #обозначаем камеру для получения картинки - 0/1
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #переводим картинку в другой цветовой диапозон
    mask = cv2.inRange(hsv, (0, 141, 91), (6, 255, 193)) #применяем маску для цвета столбов/препятствий
    mask = cv2.blur(mask, (4, 4)) #делаем размытие картинки
    contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) #ищем контуры
    contours = contours[0]
    if contours:
        x, y, w, h = cv2.boundingRect(contours[0]) #берем координаты контура
        if (x >= 0) and (x < 214): #выесняем в зависимости от контура, точнее его положения, в какую сторону уклоняться
            side = 1
        if (x >= 214) and (x < 480):
            side = 0
        if (x>= 480) and (x < 640):
            side = 0
        history = collections.deque(maxlen=10)
        val = read_distance_filtered(contours, frame, history) #проходим все фильтры
        return side, int(val) #возвращаем сторону для уклонения и дистанцию до препятствия
    else:
        return 0, 0

a, b = v_detect()
print(a, b)
