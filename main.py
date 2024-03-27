#!/usr/bin/env pybricks-micropython
##########################################################
# Algoritmo de navegación y busqueda para robot LEGO EV3 #
#                                                        #
# Autores:                                               #
# - Cristian Anjari                                      #
# - Marcos Medina                                        #
#                                                        #
# Para proyecto de Tesis 2024                            #
#                                                        #
# Universidad de Santiago de Chile                       #
# Facultad de Ciencia                                    #
#                                                        #
# Licenciatura en Ciencia de la Computación/             #
# Analista en Computación Científica                     #
#                                                        #
# Santiago, Chile                                        #
# 25/03/2024                                             #
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

#################################################

ROBOT_SPEED = 50 #Velocidad al avanzar
turn_speed_axis = 30 #Velocidad de giro en eje
turn_speed_turn = 45 #Velocidad de giro en curva

rec_dist = 800 #Distancia a recorrer por el robot

circunferencia_rueda = 56 * 3.14159
#Contadores de objetos
red_count = 0
green_count = 0
#Cuadrante
cuad = 0
#Coordenadas
red = []
green = []

"""
    Esta función permite al robot avanzar en línea recta utilizando el sensor giroscópico para mantener la dirección.
    Durante el avance, el robot detecta objetos y guarda su coordenada discriminando si son rojos o verdes.

    Parámetros:
    distance (int): La distancia que el robot debe recorrer.
    is_second_run (bool): Un indicador de si el robot está en una vuelta par del recorrido.
"""
def gyro_straight(distance, is_second_run=False):
    gyro.reset_angle(0)
    robot.reset() # Reset robot
    global red_count, green_count
    red_detected = False
    green_detected = False

    # Mientras la distancia recorrida por el robot sea menor o igual a la distancia objetivo
    while robot.distance() <= distance:
        correction = (0 - gyro.angle())*3 # Calcula la correccion
        robot.drive(ROBOT_SPEED, correction) # Conduce el robot

        # Si el sensor de color detecta un objeto rojo
        if color_sensor.color() == Color.RED and not red_detected:
            red_count = red_count + 1
            ev3.screen.print("Red:",red_count)
            red_detected = True # Marca la flag de objeto rojo detectado
            #print("Estado red:", red_detected)

            # Si es una vuelta par en el recorrido
            if is_second_run:
                print("SEGUNDA VUELTA!")
                distancia_actual = object_distance() # Obtenemos la distancia actual recorrida por el robot
                distancia_restante = distance - distancia_actual # Calculamos la distancia restante
                distancia_restante = distancia_restante - 20 # Y le restamos el tamaño del robot
                #print("dist_rest", distancia_restante)
                object_coords(distancia_restante,Red=True) # Guardamos las coordenadas del objeto rojo
            # Si es una vuelta impar
            else:
                distancia_actual = object_distance() # Obtenemos la distancia actual recorrida por el robot
                object_coords(distancia_actual,Red=True) # Guardamos las coordenadas del objeto rojo
        
        # Si el sensor de color detecta un objeto verde
        if color_sensor.color() == Color.GREEN and not green_detected:
            green_count = green_count + 1
            ev3.screen.print("Green:",green_count)
            green_detected = True # Marca la flag de objeto verde detectado
            #print("Estado green:", green_detected)

            # Si es una vuelta par en el recorrido
            if is_second_run:
                print("SEGUNDA VUELTA!")
                distancia_actual = object_distance() # Obtenemos la distancia actual recorrida por el robot
                distancia_restante = distance - distancia_actual # Calculamos la distancia restante
                distancia_restante = distancia_restante - 20 # Y le restamos el tamaño del robot
                #print("dist_rest", distancia_restante)
                object_coords(distancia_restante,Green=True) # Guardamos las coordenadas del objeto verde
            # Si es una vuelta impar
            else:
                distancia_actual = object_distance() # Obtenemos la distancia actual recorrida por el robot
                object_coords(distancia_actual,Green=True) # Guardamos las coordenadas del objeto verde

        # Si deja de detectar un objeto rojo o verde, reinicia las flags
        elif color_sensor.color() == Color.BLUE or color_sensor.color() == Color.WHITE:
            red_detected = False
            green_detected = False

    robot.stop()
    left_motor.brake()
    right_motor.brake()
#################################################

"""
    Esta función ayuda a calcular la distancia total recorrida por el robot, utilizando el promedio de las vueltas 
    de los motores y la circunferencia de las ruedas.

    Devuelve:
    float: La distancia total recorrida por el robot.
"""
def object_distance():
    return (left_motor.angle() + right_motor.angle()) * 0.5 * (circunferencia_rueda / 360.0)
#################################################
    
"""
    Esta función guarda las coordenadas de los objetos detectados.

    Parámetros:
    distancia (int): La distancia al objeto detectado.
    Red (bool): Un indicador de si el objeto detectado es rojo.
    Green (bool): Un indicador de si el objeto detectado es verde.
"""
def object_coords(distancia,Red=False,Green=False):
    global red,green

    distancia = math.floor(distancia/10) # Se transforma la distancia a centímetros
    
    # Si se detecta un objeto rojo
    if Red == True:
        #distancia = distancia + 10
        coordenada = (distancia,cuad)
        red.append(coordenada)
        print("Red:",red)

    # Si se detecta un objeto verde    
    if Green == True:
        #distancia = distancia
        coordenada = (distancia,cuad)
        green.append(coordenada)
        print("Green:",green)
#################################################


################ GIROS EN EJE  ##################
# DERECHA
def gyro_right_axis(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() + degrees # Calcula el angulo objetivo

    # Mientras el gyro sea menor que el angulo objetivo
    while gyro.angle() < target_angle :
        left_motor.run(turn_speed_axis) # El motor izquierdo gira hacia la derecha
        right_motor.run(-turn_speed_axis) # El motor derecho gira hacia la izquierda

    left_motor.brake()
    right_motor.brake()
    
# IZQUIERDA
def gyro_left_axis(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() - degrees # Calcula el angulo objetivo

    # Mientras el gyro sea mayor que el angulo objetivo
    while gyro.angle() > target_angle :
        right_motor.run(turn_speed_axis) # El motor derecho gira hacia la derecha
        left_motor.run(-turn_speed_axis) # El motor izquierdo gira hacia la izquierda

    left_motor.brake()
    right_motor.brake()
#################################################



############### GIROS EN CURVA  #################
# DERECHA
def gyro_right_turn(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() + degrees # Calcula el angulo objetivo

    # Mientras el gyro sea menor que el angulo objetivo
    while gyro.angle() < target_angle :
        left_motor.run(turn_speed_turn) # El motor izquierdo gira hacia la derecha

    left_motor.brake()

# IZQUIERDA
def gyro_left_turn(degrees):
    gyro.reset_angle(0)
    target_angle = gyro.angle() - degrees # Calcula el angulo objetivo

    # Mientras el gyro sea mayor que el angulo objetivo
    while gyro.angle() > target_angle :
        right_motor.run(turn_speed_turn) # El motor derecho gira hacia la derecha

    right_motor.brake()
#################################################


#################################################
##----------------- main ----------------------##
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
        cuad = cuad + 17
        gyro_straight(rec_dist)
        gyro_left_axis(90)
        gyro_straight(170)
        gyro_left_axis(90)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        print("1")
        wait(15000)
        ev3.speaker.beep()
        wait(2000)
        gyro.reset_angle(0)

        #segunda vuelta
        cuad = cuad + 17
        gyro_straight(rec_dist,is_second_run=True)
        gyro_right_axis(90)
        gyro_straight(170)
        gyro_right_axis(90)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        print("2")
        wait(15000)
        ev3.speaker.beep()
        wait(2000)
        gyro.reset_angle(0)
        

        #tercera vuelta
        cuad= cuad + 17
        gyro_straight(rec_dist)
        gyro_left_axis(90)
        gyro_straight(170)
        gyro_left_axis(90)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        print("3")
        wait(15000)
        ev3.speaker.beep()
        wait(2000)
        gyro.reset_angle(0)

        #cuarta vuelta
        cuad= cuad + 17
        gyro_straight(rec_dist,is_second_run=True)
        gyro_right_axis(90)
        gyro_straight(170)
        gyro_right_axis(90)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        print("4")
        wait(15000)
        ev3.speaker.beep()
        wait(2000)
        gyro.reset_angle(0)

        #quinta 
        cuad= cuad + 17
        gyro_straight(rec_dist)
        gyro_left_axis(90)
        gyro_straight(170)
        gyro_left_axis(90)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        print("5")
        wait(15000)
        ev3.speaker.beep()
        wait(2000)
        gyro.reset_angle(0)

        #sexta
        cuad= cuad + 17
        gyro_straight(rec_dist,is_second_run=True)
        gyro_right_axis(90)
        gyro_straight(170)
        gyro_right_axis(90)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        print("6")
        wait(15000)
        ev3.speaker.beep()
        wait(2000)
        gyro.reset_angle(0)

        #septima
        cuad= cuad + 17
        gyro_straight(rec_dist)
        gyro_left_axis(90)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        print("7")
        gyro.reset_angle(0)
        gyro_straight(220)
    

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