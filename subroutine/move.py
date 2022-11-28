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

def findBox(robot, sUltra, sTRight, sTLeft, DRIVE_SPEED):
    robot.drive(DRIVE_SPEED, 0)
    while True:
        if sUltra.distance < 500:
            robot.stop()

def move_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)
    DRIVE_SPEED = 300
    robot.settings(DRIVE_SPEED, 200, 200, 200)
    dir = "n"
    #robot.drive(DRIVE_SPEED, 0)
    while not (sTRight.pressed() and sTLeft.pressed()):
        ev3.screen.print(sUltra.distance())
        distToCenter = sUltra.distance() - 200
        robot.turn(distToCenter / 3)
        robot.straight(400 - distToCenter)
        robot.turn( - (distToCenter / 3))
    wallTurn(robot, DRIVE_SPEED)
    findBox(robot, sUltra, sTRight, sTLeft, DRIVE_SPEED)

    
    # if dir != "c" and sUltra.distance() < 250:
    #         ev3.screen.print("too close")
    #         dir = "c"
    #         robot.drive(DRIVE_SPEED, -20)
    #         robot.reset()
    #     if dir != "f" and sUltra.distance() > 250:
    #         ev3.screen.print("too far")
    #         dir = "f"
    #         robot.drive(DRIVE_SPEED, 20)
    #         robot.reset()
    #     if robot.angle() > 50:
    #         if dir == "f": robot.turn(-50)
    #         if dir == "c": robot.turn(50)
    #         robot.drive(DRIVE_SPEED, 0)