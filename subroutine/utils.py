#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def straight(ev3, robot, distance):
    if distance > 0:
        DRIVE_SPEED = 150
    else:
        DRIVE_SPEED = -150

    robot.reset()
    robot.drive(DRIVE_SPEED, 0)
    while abs(robot.distance()) < abs(distance):
        if Button.LEFT in ev3.buttons.pressed():
            robot.stop()
            return True
    robot.stop()
    return False