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

def turn_left_triangle(robot):
    DRIVE_SPEED = 150

    robot.drive(-DRIVE_SPEED, 40)
    time.sleep(1)
    robot.drive(DRIVE_SPEED, -130)
    time.sleep(1)
    robot.drive(-DRIVE_SPEED, 0)
    time.sleep(0.5)

    robot.stop()

def search_main(ev3, mLeft, mRight, sColor):
    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=125)

    while True:             # Testing
        turn_left_triangle(robot)
        time.sleep(3)

    ROOM_LENGTH = 960   # in millimeters
    ROBOT_WIDTH = 125   # in millimeters

    robot.straight(ROOM_LENGTH)
    turn_left_triangle(robot)
    robot.straight(ROOM_LENGTH - ROBOT_WIDTH)
    turn_left_triangle(robot)
    robot.straight(ROOM_LENGTH - ROBOT_WIDTH)
    turn_left_triangle(robot)
    robot.straight(ROOM_LENGTH - 2 * ROBOT_WIDTH)
    turn_left_triangle(robot)
    for i in range(1, math.ceil(ROOM_LENGTH/ROBOT_WIDTH/2)):
        robot.straight(ROOM_LENGTH - 2*i * ROBOT_WIDTH)
        robot.turn(-90)
        robot.straight(ROOM_LENGTH - (2*i+1) * ROBOT_WIDTH)
        robot.turn(-90)
        robot.straight(ROOM_LENGTH - (2*i+1) * ROBOT_WIDTH)
        robot.turn(-90)
        robot.straight(ROOM_LENGTH - 2*(i+1) * ROBOT_WIDTH)
        robot.turn(-90)