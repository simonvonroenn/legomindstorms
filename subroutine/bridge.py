#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


def bridge_main(ev3, mLeft, mRight, mSensor, sColor, sInfra, sTouch1, sTouch2):

    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)
    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100

    count = 0
    boost = 0
    while True:
        if sColor.color() == None:
            robot.stop()
            robot.straight(-200)
            robot.turn(270)
        robot.drive(DRIVE_SPEED + boost, 0)
        if count < 1000:
            count+=1
        else:
            ev3.screen.print(robot.distance())
            boost = 100 - robot.distance()
            count = 0
            robot.reset()
