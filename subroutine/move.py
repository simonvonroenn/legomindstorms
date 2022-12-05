#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def driveToWall(ev3, robot, sTRight, sTLeft, sUltra):
    CENTER_DISTANCE = 200
    DRIVE_UNIT = 200
    ADJUSTMENT_PER_UNIT = 20
    ADJUSTMENT_PRECISION = 3
    LEVEL_OF_SIGNIFICANCE = 20
    adjusted = False
    orientate = 0
    while not (sTRight.pressed() and sTLeft.pressed()):
        prevDist = sUltra.distance() - CENTER_DISTANCE
        distToCenter = prevDist
        if abs(distToCenter) < LEVEL_OF_SIGNIFICANCE: adjusted = True
        if not adjusted:
            robot.stop()
            ev3.screen.print("distToCenter: " + str(distToCenter))
            robot.turn(distToCenter / ADJUSTMENT_PRECISION + orientate)
            robot.straight(DRIVE_UNIT)
            robot.turn( - (distToCenter / ADJUSTMENT_PRECISION))
            distToCenter = sUltra.distance() - CENTER_DISTANCE
            orientate = distToCenter - prevDist + (ADJUSTMENT_PER_UNIT if distToCenter > CENTER_DISTANCE else -ADJUSTMENT_PER_UNIT)
            ev3.screen.print("diff: " + str(orientate))
        else:
            robot.drive(DRIVE_SPEED, 0)
        

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