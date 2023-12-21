#!/usr/bin/env pybricks-micropython
##########################################################
# Algoritmo de navegación y busqueda para robot LEGO EV3 #
#                                                        #
# Autores:                                               #
# - Cristian Anjari                                      #
# - Marcos Medina                                        #
#                                                        #
# Para proyecto de tesis 2023                            #
#                                                        #
# Universidad de Santiago de Chile                       #
# Facultad de Ciencia                                    #
# Licenciatura en Ciencia de la Computación/             #
# Analista en Computación Científica                     #
#                                                        #
# Santiago, Chile                                        #
# 29/11/2023                                             #
##########################################################

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
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
ROBOT_SPEED = 70
circunferencia_rueda = 56 * 3.14159
#Contadores de objetos
red_count = 0
blue_count = 0
#Cuadrante
cuad = 0
#Coordenadas
red = []
blue = []

# Gyro straight
def gyro_straight(distance, is_second_run=False):
    gyro.reset_angle(0)
    robot.reset() # Reset robot
    global red_count, blue_count
    red_detected = False
    blue_detected = False

    while robot.distance() <= distance:
        correction = (0 - gyro.angle())*3 # Calcula la correccion
        robot.drive(ROBOT_SPEED, correction) # Conduce el robot

        #Detecta rojo!
        if color_sensor.color() == Color.RED and not red_detected:
            red_count = red_count + 1
            #print("Red:",red_count)
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
        
        #Detecta Azul!
        if color_sensor.color() == Color.BLUE and not blue_detected:
            blue_count = blue_count + 1
            #print("Blue:",blue_count)
            blue_detected = True
            #print("Estado blue:", blue_detected)

            if is_second_run:
                print("SEGUNDA VUELTA!")
                distancia_actual = object_distance()
                distancia_restante = distance - distancia_actual
                #print("dist_rest", distancia_restante)
                object_coords(distancia_restante,Blue=True)
            else:
                distancia_actual = object_distance()
                object_coords(distancia_actual,Blue=True)

        elif color_sensor.color() == Color.WHITE or color_sensor.color() == Color.BLACK:
            red_detected = False

    robot.stop()
    left_motor.brake()
    right_motor.brake()
    
def object_distance():
    # Calcular la distancia total recorrida por el robot
    distancia_total = (left_motor.angle() + right_motor.angle()) * 0.5 * (circunferencia_rueda / 360.0)
    return distancia_total
    
    
def object_coords(distancia,Red=False,Blue=False):
    global red,blue

    coordenada = (distancia,cuad)
    
    if Red == True:
        red.append(coordenada)
        print("Red:",red)
    
    if Blue == True:
        blue.append(coordenada)
        print("Blue:",blue)

############# GIROS EN EJE #############
# DERECHA
def gyro_right_axis(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() + degrees

    while gyro.angle() < target_angle: # Si el gyro es menor que los angulo objetivo
        left_motor.run(70) # Gira a la derecha
        right_motor.run(-70)

    left_motor.brake()
    right_motor.brake()
    
# IZQUIERDA
def gyro_left_axis(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() - degrees

    while gyro.angle() > target_angle: # Si el gyro es mayor que el angulo objetivo
        right_motor.run(70) # Gira a la izquierda
        left_motor.run(-70)

    left_motor.brake()
    right_motor.brake()
##########################################


############# GIROS EN CURVA #############
# DERECHA
def gyro_right_turn(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() + degrees

    while gyro.angle() < target_angle: # Si el gyro es menor que los angulo objetivo
        left_motor.run(100) # Gira a la derecha

    left_motor.brake()

# IZQUIERDA
def gyro_left_turn(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() - degrees

    while gyro.angle() > target_angle: # Si el gyro es mayor que el angulo objetivo
        right_motor.run(100) # Gira a la izquierda

    right_motor.brake()
##########################################
rec_dist = 750

#primera vuelta
cuad= cuad + 1
gyro_straight(rec_dist)
gyro_left_turn(90)
gyro_straight(140)
gyro_left_axis(90)
left_motor.reset_angle(0)
right_motor.reset_angle(0)
print("1")

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