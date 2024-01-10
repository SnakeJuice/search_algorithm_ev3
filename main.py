#!/usr/bin/env pybricks-micropython
##########################################################
# Algoritmo de navegación y busqueda para robot LEGO EV3 #
#                                                        #
# Autores:                                               #
# - Cristian Anjari                                      #
# - Marcos Medina                                        #
#                                                        #
# Para proyecto de Tesis 2023                            #
#                                                        #
# Universidad de Santiago de Chile                       #
# Facultad de Ciencia                                    #
#                                                        #
# Licenciatura en Ciencia de la Computación/             #
# Analista en Computación Científica                     #
#                                                        #
# Santiago, Chile                                        #
# 03/01/2024                                             #
##########################################################

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.messaging import BluetoothMailboxClient, TextMailbox

import math

# Inicializa el ladrillo EV3 y los motores
ev3 = EV3Brick()
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

# Configura el sensor de color en un puerto adecuado
color_sensor = ColorSensor(Port.S2)

# Configura la unidad base (drive base) con los motores
gyro = GyroSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=125)

##########################################
ROBOT_SPEED = 30 #Velocidad al avanzar
turn_speed_axis = 25 #Velocidad de giro en eje
turn_speed_turn = 40 #Velocidad de giro en curva

rec_dist = 750 #Distancia a recorrer por el robot

circunferencia_rueda = 56 * 3.14159
#Contadores de objetos
red_count = 0
green_count = 0
#Cuadrante
cuad = 0
#Coordenadas
red = []
green = []

# Gyro straight
def gyro_straight(distance, is_second_run=False):
    gyro.reset_angle(0)
    robot.reset() # Reset robot
    global red_count, green_count
    red_detected = False
    green_detected = False

    while robot.distance() <= distance:
        correction = (0 - gyro.angle())*3 # Calcula la correccion
        robot.drive(ROBOT_SPEED, correction) # Conduce el robot

        #Detecta Rojo!
        if color_sensor.color() == Color.RED and not red_detected:
            red_count = red_count + 1
            ev3.screen.print("Red:",red_count)
            red_detected = True
            #print("Estado red:", red_detected)

            if is_second_run:
                print("SEGUNDA VUELTA!")
                distancia_actual = object_distance()
                distancia_restante = distance - distancia_actual
                #print("dist_rest", distancia_restante)
                object_coords(distancia_restante,Red=True)
            else:
                distancia_actual = object_distance()
                object_coords(distancia_actual,Red=True)
        
        #Detecta Verde!
        if color_sensor.color() == Color.GREEN and not green_detected:
            green_count = green_count + 1
            ev3.screen.print("Green:",green_count)
            green_detected = True
            #print("Estado green:", green_detected)

            if is_second_run:
                print("SEGUNDA VUELTA!")
                distancia_actual = object_distance()
                distancia_restante = distance - distancia_actual
                #print("dist_rest", distancia_restante)
                object_coords(distancia_restante,Green=True)
            else:
                distancia_actual = object_distance()
                object_coords(distancia_actual,Green=True)

        elif color_sensor.color() == Color.BLUE or color_sensor.color() == Color.WHITE:
            red_detected = False
            green_detected = False

    robot.stop()
    left_motor.brake()
    right_motor.brake()
    
def object_distance():
    # Calcular la distancia total recorrida por el robot
    return (left_motor.angle() + right_motor.angle()) * 0.5 * (circunferencia_rueda / 360.0)
    
    
def object_coords(distancia,Red=False,Green=False):
    global red,green

    distancia = math.floor(distancia/10)
    coordenada = (distancia,cuad)
    
    if Red == True:
        red.append(coordenada)
        print("Red:",red)
    
    if Green == True:
        green.append(coordenada)
        print("Green:",green)


############# GIROS EN EJE #############
# DERECHA
def gyro_right_axis(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() + degrees

    while gyro.angle() < target_angle : # Si el gyro es menor que los angulo objetivo
        left_motor.run(turn_speed_axis) # Gira a la derecha
        right_motor.run(-turn_speed_axis)

    left_motor.brake()
    right_motor.brake()
    
# IZQUIERDA
def gyro_left_axis(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() - degrees

    while gyro.angle() > target_angle -1.5: # Si el gyro es mayor que el angulo objetivo
        right_motor.run(turn_speed_axis) # Gira a la izquierda
        left_motor.run(-turn_speed_axis)

    left_motor.brake()
    right_motor.brake()
##########################################



############# GIROS EN CURVA #############
# DERECHA
def gyro_right_turn(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() + degrees

    while gyro.angle() < target_angle : # Si el gyro es menor que los angulo objetivo
        left_motor.run(turn_speed_turn) # Gira a la derecha

    left_motor.brake()

# IZQUIERDA
def gyro_left_turn(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() - degrees

    while gyro.angle() > target_angle -1.5: # Si el gyro es mayor que el angulo objetivo
        right_motor.run(turn_speed_turn) # Gira a la izquierda

    right_motor.brake()
#################################################

#################################################
##----------------Partida----------------------##
#################################################

SERVER = 'Master'

client = BluetoothMailboxClient()

mbox = TextMailbox('greeting', client)

print('establishing connection...')
client.connect(SERVER)
print('connected!')

while True:
    mbox.send('Buscador conectado')
    mbox.wait()

    if(mbox.read()=='inicia buscador'):

        #primera vuelta
        cuad= cuad + 1
        gyro_straight(rec_dist)
        gyro_left_turn(90)
        gyro_straight(140)
        gyro_left_axis(90)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        print("1")

    '''
    #segunda vuelta
    cuad= cuad + 1
    gyro_straight(rec_dist)
    gyro_right_turn(90)
    gyro_straight(140)
    gyro_right_axis(90)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    print("2")

    #tercera vuelta
    cuad= cuad + 1
    gyro_straight(rec_dist)
    gyro_left_turn(90)
    gyro_straight(140)
    gyro_left_axis(90)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    print("3")

    #cuarta vuelta
    cuad= cuad + 1
    gyro_straight(rec_dist)
    gyro_right_turn(90)
    gyro_straight(140)
    gyro_right_axis(90)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    print("4")

    #quinta 
    cuad= cuad + 1
    gyro_straight(rec_dist)
    gyro_left_turn(90)
    gyro_straight(140)
    gyro_left_axis(90)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    print("5")

    #sexta
    cuad= cuad + 1
    gyro_straight(rec_dist)
    gyro_right_turn(90)
    gyro_straight(140)
    gyro_right_axis(90)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    print("6")

    #septima
    cuad= cuad + 1
    gyro_straight(rec_dist)
    gyro_left_turn(90)
    gyro_straight(140)
    gyro_left_axis(90)
    left_motor.reset_angle(0)
    right_motor.reset_angle(0)
    print("7")
    '''
    

    #Convert red and green to string
    red_string = str(red)
    green_string = str(green)

    #Send data to server
    mbox.send(red_string)

    mbox.wait_new()

    if(mbox.read()=='recibido'):
        mbox.send(green_string)

    mbox.wait_new()

    if(mbox.read()=='listo'):
        mbox.send('terminado')

    else:
        print("no se recibio") 