#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math
import time

def turn_Left(robot):
    DRIVE_SPEED = 150

    robot.drive(-DRIVE_SPEED, 20)
    time.sleep(2)
    robot.drive(DRIVE_SPEED, 65)
    time.sleep(2)
    robot.drive(-DRIVE_SPEED, 0)
    time.sleep(2)
    
    robot.stop()

def search_main(ev3, mLeft, mRight, sColor):

    ROOM_LENGTH = 1000
    ROBOT_WIDTH = 100

    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=125)
    for i in range(math.trunc(ROOM_LENGTH/ROBOT_WIDTH/2)):
        robot.straight(ROOM_LENGTH - 2*i * ROBOT_WIDTH)
        turn_Left(robot)
        robot.straight(ROOM_LENGTH - (2*i+1) * ROBOT_WIDTH)
        turn_Left(robot)
        robot.straight(ROOM_LENGTH - (2*i+1) * ROBOT_WIDTH)
        turn_Left(robot)
        robot.straight(ROOM_LENGTH - 2*(i+1) * ROBOT_WIDTH)
        turn_Left(robot)



# def search_marker(ev3, mLeft, mRight, touchL, touchR, sColor):
#     #enters on right side of wall
#     #optimal path: up; 90 deg left, short burst, 90 deg left; down; 90 deg right, short burst, 90 deg right;
#     #for up and down use drive and continually check for press of button and marker
#     #for short burst use straight and two checks for the maerker
#     #do for (measure how often)
#     robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)
#     LOOP_NUMBER = 5 #exact value would be Room_length / Robot_size 
#     i = 0
#     stop = False
#     while i < LOOP_NUMBER:
#         while not (touchL or touchR):
#             if sColot.color() == Color.RED:
#                 return
#         if turn_subroutine(robot, sColor, i):
#             return
#         i += 1

# #makes a 90 deg turn, moves forward a bit and makes another 90 deg turn; i decides if the turn is right or left
# def turn_subroutine(robot, sColor, i):
#     #positive = left, negative = right
#     angle = 90
#     if (i % 2) == 1:
#         angle = -90
#     robot.turn(angle)
#     if sColor.color() == Color.RED:
#         return True
#     robot.straight(50)
#     if sColor.color() == Color.RED:
#         return True
#     robot.turn(angle)
#     return False