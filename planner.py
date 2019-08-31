# -*- coding: utf-8 -*-

#Используемый фрэйм: BODY

import video_detect
import math
num = 0
points = []
states = []


def nav(s, m, x, y): #первод углов в первый наш фрэйм
    sina = (y-m)/math.sqrt((x-s)**2+(y-m)**2)
    cosa = (x-s)/math.sqrt((x-s)**2+(y-m)**2)
    angle = math.degrees(math.acos(cosa))
    if (sina >= 0) and (angle>=0) and (angle<=90):
        rotate = 90 - angle
    if (sina >= 0) and (angle >=90) and (angle<=180):
        rotate = 90 - angle
    if (sina < 0) and (angle>=0) and (angle<=90):
        rotate = 180 - angle
    if (sina < 0) and (angle >=90) and (angle<=180):
        rotate = -(270-angle)
    return rotate

def nav2(s, m, x, y): #во второй
    sina = (y-m)/math.sqrt((x-s)**2+(y-m)**2)
    cosa = (x-s)/math.sqrt((x-s)**2+(y-m)**2)
    angle = math.degrees(math.acos(cosa))
    if sina <= 0:
        angle = -1*angle

    return angle
def nav3(s, m, x, y): #в третий
    sina = (y-m)/math.sqrt((x-s)**2+(y-m)**2)
    f = (x-s)/math.sqrt((x-s)**2+(y-m)**2)
    if (f < 0) and (sina >= 0):
        angle = 180 - math.acos(math.fabs(f))
    if (f > 0) and (sina >= 0):
        angle = math.acos(math.fabs(f))
    if (f < 0) and (sina <= 0):
        angle = -(180 - math.acos(math.fabs(f)))
    if (f > 0) and (sina <= 0):
        angle = -1*(math.acos(math.fabs(f)))
    return(angle)

def nav4(s, m, x, y): #Перевод наиболее подходящий для нашей задачи, т.к. работает в фрэйме aruco_map
    sina = (y-m)/math.sqrt((x-s)**2+(y-m)**2)
    cosa = (x-s)/math.sqrt((x-s)**2+(y-m)**2)
    angle = math.degrees(math.acos(cosa))
    if (sina > 0) and (cosa > 0):
        angle = math.degrees(math.acos(cosa))
    if (sina>0) and (cosa<0):
        angle = 180 - math.degrees(math.acos(math.fabs(cosa)))
    if (sina < 0) and (cosa < 0):
        angle = -1*(180-math.degrees(math.acos(math.fabs(cosa))))
    if (sina < 0) and (cosa > 0):
        angle = -1*math.degrees(math.acos(cosa))
    if (sina == 0) and (cosa == 1):
        angle = 0
    if (sina == 1) and (cosa == 0):
        angle = 90
    if (sina == 0) and (cosa == -1):
        angle = 180
    if (sina == -1) and (cosa == 0):
        angle = -90
    return angle

def reader(): #функция по чтению точек из points.txt
    file = open('points.txt', 'r')
    for line in file.readlines():
        points.append(line)
    file.close()
    return points

def check_env():
    #Тут идет полная проверка как с вебки, так и с сонаров и в итоге должен выйти ответ в return:
    #повернут налево или направо или назад, а также с каким ускорением
    #важно помнить, что частота цикла может быть 10Гц, значит указывать ускорения
    #большие не надо, т.к. коптер еще успеет увернуться.
    side, dist = video_detect.v_detect()
    return int(side), int(dist)

