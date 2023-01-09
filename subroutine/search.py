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
    DRIVE_SPEED = 100

    robot.stop()
    robot.drive(-DRIVE_SPEED, 40)
    time.sleep(1)
    robot.drive(DRIVE_SPEED, -140)  # mathematically it should be -130°
    time.sleep(1)
    robot.stop()

def search_main(ev3, mLeft, mRight, sColor):
    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=135)
    DRIVE_SPEED = 150

    # while True:             # Testing
    #     turn_left_triangle(robot)
    #     #robot.turn(-90)
    #     time.sleep(1000)

    ROOM_LENGTH = 900   # in millimeters
    SPOT_WIDTH = 70     # in millimeters
    TURN_OFFSET = 100

    # robot is driving too long distances after second turn

    robot.drive(DRIVE_SPEED, 0)
    time.sleep((ROOM_LENGTH + 50)/DRIVE_SPEED)
    turn_left_triangle(robot)
    robot.drive(DRIVE_SPEED, 0)
    time.sleep((ROOM_LENGTH - SPOT_WIDTH - TURN_OFFSET)/DRIVE_SPEED)
    turn_left_triangle(robot)
    robot.drive(DRIVE_SPEED, 0)
    time.sleep((ROOM_LENGTH - SPOT_WIDTH - TURN_OFFSET)/DRIVE_SPEED)
    turn_left_triangle(robot)
    robot.drive(DRIVE_SPEED, 0)
    time.sleep((ROOM_LENGTH - 2 * SPOT_WIDTH - TURN_OFFSET)/DRIVE_SPEED)
    turn_left_triangle(robot)
    for i in range(1, math.trunc(ROOM_LENGTH/SPOT_WIDTH/2)):
        robot.straight(ROOM_LENGTH - 2*i * SPOT_WIDTH)
        robot.turn(-90)
        robot.straight(ROOM_LENGTH - (2*i+1) * SPOT_WIDTH)
        robot.turn(-90)
        robot.straight(ROOM_LENGTH - (2*i+1) * SPOT_WIDTH)
        robot.turn(-90)
        robot.straight(ROOM_LENGTH - 2*(i+1) * SPOT_WIDTH)
        robot.turn(-90)