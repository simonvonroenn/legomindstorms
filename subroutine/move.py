#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def wallTurn(robot, DRIVE_SPEED):
    robot.stop()
    robot.straight(-100)
    robot.turn(580)
    robot.drive(DRIVE_SPEED, 0)

def findBox(robot, sUltra, sTRight, sTLeft, DRIVE_SPEED):
    robot.drive(DRIVE_SPEED, 0)
    while True:
        if sTRight.pressed() or sTLeft.pressed:
            wallTurn(robot, DRIVE_SPEED)

def move_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)
    DRIVE_SPEED = 100
    findBox(robot, sUltra, sTRight, sTLeft, DRIVE_SPEED)