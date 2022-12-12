#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time

def orientate(robot, sUltra):
    ANGLE_OFFSET = -60
    dist = sUltra.distance()
    while True:
        dist = sUltra.distance() if sUltra.distance() < dist else dist
        robot.turn(5)
        if sUltra.distance() - dist > 5: break
    while True:
        dist = sUltra.distance() if sUltra.distance() < dist else dist
        robot.turn(-5)
        if sUltra.distance() - dist > 5: break
    robot.turn(ANGLE_OFFSET)

def driveToWall(ev3, robot, sTRight, sTLeft, sUltra, DRIVE_SPEED):
    CENTER_DISTANCE = 30
    DRIVE_UNIT = 200
    ADJUSTMENT_PRECISION = 2.5
    TOLERANCE = 30
    adjusted = False
    while not (sTRight.pressed() or sTLeft.pressed()):
        prevDist = sUltra.distance() - CENTER_DISTANCE
        distToCenter = prevDist
        if abs(distToCenter) < TOLERANCE: adjusted = True
        if not adjusted:
            robot.stop()
            ev3.screen.print(distToCenter)
            robot.turn(distToCenter / ADJUSTMENT_PRECISION)
            robot.straight(DRIVE_UNIT)
            robot.turn( - (distToCenter / ADJUSTMENT_PRECISION))
        else:
            robot.drive(DRIVE_SPEED, 0)

def findBox(robot, sUltra, DRIVE_SPEED):
    robot.drive(-DRIVE_SPEED, 0)
    while True:
        if sUltra.distance() < 500:
            robot.stop()
            robot.straight(-50)
            break

def moveBoxToWall(robot, DRIVE_SPEED, NINETY_DEGREES):
    robot.turn(-NINETY_DEGREES)
    robot.reset()
    robot.drive(DRIVE_SPEED, 0)
    while robot.distance() < 5:
        robot.reset()
        time.sleep(1)
    robot.stop()
    robot.straight(-100)
    robot.turn(-NINETY_DEGREES)
    robot.straight(250)
    robot.turn(NINETY_DEGREES)
    robot.straight(200)
    robot.turn(NINETY_DEGREES)
    robot.drive(DRIVE_SPEED, 0)
    while robot.distance() < 5:
        robot.reset()
        time.sleep(1)

def move_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)
    DRIVE_SPEED = 400
    NINETY_DEGREES = 250
    robot.settings(DRIVE_SPEED, 200, 200, 200)
    # while True:
    #     ev3.screen.print(sUltra.distance())
    orientate(robot, sUltra)
    driveToWall(ev3, robot, sTRight, sTLeft, sUltra, DRIVE_SPEED)
    mSensor.run_target(50, -90)
    findBox(robot, sUltra, DRIVE_SPEED)
    moveBoxToWall(robot, DRIVE_SPEED, NINETY_DEGREES)