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

robot = DriveBase(left_motor, right_motor, wheel_diameter=52, axle_track=125)

##########################################
circunferencia_rueda = 56 * 3.14159
count = 0
cuad = 0
ubicaciones = []

# Gyro straight
def gyro_straight(distance, is_second_run=False):
    target = gyro.angle() # Target angle
    gain = 7 # Gain
    robot.reset() # Reset robot
    global count
    red_detected = False

    while robot.distance() < distance: # Si la distancia actual es menor que la distancia objetivo hasta llegar.
        correction = target - gyro.angle() # Calcula la correccion
        turn_power = correction * gain # Calcula la potencia de giro
        robot.drive(60, turn_power) # Conduce el robot
        
        # Verificar si el color es rojo durante el avance
        if color_sensor.color() == Color.RED and not red_detected:
            count = count + 1
            red_detected = True
            
            if is_second_run:
                ev3.screen.print("SEGUNDA VUELTA!")
                distancia_actual = object_distance()
                distancia_restante = distance - distancia_actual
                #ev3.screen.print("dist_rest", distancia_restante)
            else:
                pass
                object_distance()
            
        elif color_sensor.color() == Color.WHITE or color_sensor.color() == Color.BLACK:
            red_detected = False
   
    robot.stop()
    
def object_distance():
    # Calcular la distancia total recorrida por el robot
    distancia_total = (left_motor.angle() + right_motor.angle()) * 0.5 * (circunferencia_rueda / 360.0)
    # Posición relativa
    #print("Dist_rojo:", distancia_total)
    coordenada = (distancia_total,cuad)
    ubicaciones.append(coordenada)
    #ev3.screen.print(ubicaciones)
    ev3.screen.print("[",distancia_total,"\n",cuad,"]")
    #print("Cant_rojos:", count)
    
    return distancia_total
    

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

#primera vuelta
cuad= cuad + 1
gyro_straight(1070)
gyro_right_turn(90)
gyro_straight(140)
gyro_right_axis(90)
left_motor.reset_angle(0)
right_motor.reset_angle(0)

#segunda vuelta
cuad= cuad + 1
gyro_straight(1000, is_second_run=True)
gyro_left_turn(90)
gyro_straight(140)
gyro_left_axis(90)
left_motor.reset_angle(0)
right_motor.reset_angle(0)

#tercera vuelta
cuad= cuad + 1
gyro_straight(1070)
gyro_right_turn(90)
gyro_straight(130)
gyro_right_axis(90)
left_motor.reset_angle(0)
right_motor.reset_angle(0)

#ultima vuelta
cuad= cuad + 1
gyro_straight(1000, is_second_run=True)
gyro_left_turn(90)
gyro_straight(140)
gyro_left_axis(90)
left_motor.reset_angle(0)
right_motor.reset_angle(0)