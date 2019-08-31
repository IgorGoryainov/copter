# -*- coding: utf-8 -*-

import rospy
import time
import math
from clever import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
import planner
import RPi.GPIO as GPIO

'''В points.txt пишем в следующем формате: 
    
    1.5 2 1.5 0 
    2 3 1.5 1
    координаты x, y, z точки и state (необходимо ли, когда коптер долетит до этой точки сделать небольшую посадку, чтобы сбросить груз, а потом дальше 
    взлететь и продолжить выполнять полет по точкам из points.txt
'''
rospy.init_node('flight')
num = 0 #счетчик кол-ва точек удачно завершенных
end_num = 1 #необходимо указать сколько всего точек полета в points.txt
points = [] #массив хронящий в себе точки полета
r = rospy.Rate(10) #подобие тай слип в цикле, подробнее смотреть на гитбуке, в описании полета по окружности
tolerance = 0.8 #точность в м, необходимая для срабатывания счетчика, что коптер долетел до точки
points = planner.reader() #запускаем функцию по считванию всех точек в points.txt
state = 0 #переменная необходима для того, чтобы можно было делать посадку, не выходя из цикла, чтобы взять груз и продолжить полет.

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT) #махинации для работы электромагнита


arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

rospy.init_node('flight')

navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
time.sleep(5) #совершаем взлет, сразу после старта программы

GPIO.output(18, GPIO.HIGH) #сразу подаем напряжение на магнит (открываем реле)

while not rospy.is_shutdown():
    x, y, z, state = map(float, points[num].split(' ')) #считываем значения определенной по порядку точки
    state = int(state)
    telemetry = get_telemetry(frame_id='aruco_map')
    s, m, h = telemetry.x, telemetry.y, telemetry.z #получаем собственные координаты
    side, distance = planner.check_env() #получаем данные в виде: на каком расстояние препятствие и в какую сторону необходио уклониться
    if distance < 60: #данное неравенство корректировать в зависимости от тестов
        if side == 0:
            set_position(x=0, y=1, z=0, frame_id='body')
            time.sleep(2)
        if side == 1:
            set_position(x=0, y=-1, z=0, frame_id='body')
            time.sleep(2)
        if side == 2:
            set_position(x=-1, y=0, z=0, frame_id='body')
            time.sleep(2)
        # 0-влево, 1-вправо, 2 - назад'''
    angle = planner.nav4(s, m, x, y) #получаем угол в фрэйме арукомап, на который необходимо ориентироватсья коптеру для следования точно на точку
    set_position(x = telemetry.x, y = telemetry.y, z=1.5, yaw=math.radians(angle), frame_id='aruco_map') #доворачиваемся, сохраняя коорлинаты
    set_position(x=1, y=0, z=telemetry.z, yaw=float('nan'), frame_id='body') #движемся вперед
    if math.sqrt((x-s)**2 + (y-m)**2 + (z-1.5)**2) < tolerance: #проверяем, не долетели ли до точки
        num+=1 #если долетели, повышаем счетчик
        if state == 1: #про переменную state можно прочитать выше; кратко - проверяем указана ли в points.txt необходимость сесть в данной точке и сбросить груз.
            set_position(x=0, y=0, z=-2, frame_id='body')
            time.sleep(4)
            arming(False)
            GPIO.output(18, GPIO.LOW)
            time.sleep(5) #время на сброс груза
            navigate(x=0, y=0, z=1.5, speed=0.5, frame_id='body', auto_arm=True)
            time.sleep(5)
    if num >= end_num: #если все точки успешно закончены, то делаем посадку
        print('to land')
        break
    side, distance = 0, 0
    r.sleep()

set_position(x=0, y=0, z=-2, frame_id='body')
time.sleep(4)
telem = get_telemetry(frame_id='aruco_map')
print(telem.x, telem.y)
arming(False)
