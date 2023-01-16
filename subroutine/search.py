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

def searchSpot(robot, sColor, ROOM_LENGTH, SPOT_WIDTH, time, TURN_OFFSET):
    robot.reset()
    while robot.distance() < (ROOM_LENGTH - time * SPOT_WIDTH - TURN_OFFSET):
        if sColor.color() == Color.RED:
            robot.stop()
            return True
    return False

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
    TURN_OFFSET = 150

    # robot is driving too long distances after second turn

    # 1
    robot.drive(DRIVE_SPEED, 0)
    time.sleep(1) # Prevents scanning the blue line. '-150' in searchSpot() to counterbalance this sleep
    if searchSpot(robot, sColor, ROOM_LENGTH + 50 - 150, 0, 1, 0):
        return
    turn_left_triangle(robot)
    # 2
    robot.drive(DRIVE_SPEED, 0)
    if searchSpot(robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 1, TURN_OFFSET):
        return
    turn_left_triangle(robot)
    # 3
    robot.drive(DRIVE_SPEED, 0)
    if searchSpot(robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 1, TURN_OFFSET):
        return
    turn_left_triangle(robot)
    # 4
    robot.drive(DRIVE_SPEED, 0)
    if searchSpot(robot, sColor, ROOM_LENGTH, 2 * SPOT_WIDTH, 1, TURN_OFFSET):
        return
    turn_left_triangle(robot)

    for i in range(2, math.trunc(ROOM_LENGTH/SPOT_WIDTH/2)):
        # 1
        robot.drive(DRIVE_SPEED, 0)
        if searchSpot(robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 2*i, 0):
            return
        robot.stop()
        robot.turn(-90)
        # 2
        robot.drive(DRIVE_SPEED, 0)
        if searchSpot(robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 2*i+1, 0):
            return
        robot.stop()
        robot.turn(-90)
        # 3
        robot.drive(DRIVE_SPEED, 0)
        if searchSpot(robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 2*i+1, 0):
            return
        robot.stop()
        robot.turn(-90)
        # 4
        robot.drive(DRIVE_SPEED, 0)
        if searchSpot(robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 2*(i+1), 0):
            return
        robot.stop()
        robot.turn(-90)