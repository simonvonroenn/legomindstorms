#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


def bridge_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):

    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)

    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100

    # Variables for adjusting speed
    count = 0
    boost = 0

    # Takes a step back and turns the robot by 180°
    #robot.straight(-200)
    #robot.turn(580)

    # Orientates the Infrared Sensor correctly
    #mSensor.run_target(20, 90)

    while sColor.color() != Color.BLUE:
        # Checks if there is an abyss and turns left if so
        if sUltra.distance() > 100:
            robot.stop()
            robot.straight(150)
            robot.turn(290)
        # Drives
        robot.drive(-1 * (DRIVE_SPEED + boost), 0)
        ev3.screen.print(sUltra.distance())
        # Changes the speed based on the slope of the track
        if count < 1000:
            count+=1
        else:
            boost = 100 - robot.distance()
            count = 0
            robot.reset()

    robot.stop()

    # Undo Infrared Sensor orientation
    #mSensor.run_target(10, -90)

    # Undo robot rotation
    #robot.turn(580)
