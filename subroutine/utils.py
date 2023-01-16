#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def straight(ev3, robot, distance):
    robot.reset()
    robot.drive()
    while robot.distance() < distance:
        if Button.LEFT in ev3.buttons.pressed():
            robot.stop()
            return True
    robot.stop()
    return False