#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def driveToWall(ev3, robot, sTRight, sTLeft, sUltra):
    adjusted = False
    while not (sTRight.pressed() and sTLeft.pressed()):
        prevDist = sUltra.distance() - 200
        distToCenter = prevDist
        if abs(distToCenter) < 20: adjusted = True
        if not adjusted:
            robot.stop()
            ev3.screen.print(distToCenter)
            robot.turn(distToCenter / 2 + orientate)
            robot.straight(200)
            robot.turn( - (distToCenter / 2))
        else:
            robot.drive(DRIVE_SPEED, 0)
        distToCenter = sUltra.distance() - 200
        orientate = distToCenter - prevDist + 20
        

def wallTurn(robot):
    robot.stop()
    robot.straight(-100)
    robot.turn(580)

def findBox(robot, sUltra, DRIVE_SPEED):
    robot.drive(DRIVE_SPEED, 0)
    while True:
        if sUltra.distance() < 500:
            robot.stop()

def moveBoxToWall(robot):
    robot.turn(290)
    robot.straight(1000)
    # to be implemented

def move_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)
    DRIVE_SPEED = 400
    robot.settings(DRIVE_SPEED, 200, 200, 200)
    driveToWall(ev3, robot, sTRight, sTLeft, sUltra)
    wallTurn(robot, DRIVE_SPEED)
    findBox(robot, sUltra, sTRight, sTLeft, DRIVE_SPEED)
    moveBoxToWall(robot)