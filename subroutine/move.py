#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def driveToWall(ev3, robot, sTRight, sTLeft, sUltra, mSensor, DRIVE_SPEED):
    ev3.screen.print("driveToWall")
    CENTER_DISTANCE = 100
    DRIVE_UNIT = 80
    ADJUSTMENT_PRECISION = 4
    TOLERANCE = 10

    mSensor.run_target(100, -90)
    while True:
        prevDist = sUltra.distance() - CENTER_DISTANCE
        distToCenter = prevDist
        if abs(distToCenter) < TOLERANCE: break
        robot.turn(distToCenter / ADJUSTMENT_PRECISION)
        robot.straight(DRIVE_UNIT)
        robot.turn( - (distToCenter / ADJUSTMENT_PRECISION))
    
    mSensor.run_target(100, 0)
    robot.drive(DRIVE_SPEED + 100, -1)
    while True:
        if sTRight.pressed() or sTLeft.pressed():
            break

def findBox(ev3, robot, sUltra, mSensor, DRIVE_SPEED):
    ev3.screen.print("findBox")
    robot.stop()
    robot.straight(-50)
    robot.turn(-180)
    mSensor.run_target(100, -90)
    robot.drive(DRIVE_SPEED, 0)
    while True:
        if sUltra.distance() < 500:
            robot.stop()
            robot.straight(200)
            break
    mSensor.run_target(100, 0)

def moveBoxToCorner(ev3, robot):
    ev3.screen.print("moveBoxToCorner")
    robot.turn(90)
    robot.straight(800)
    robot.straight(-50)
    robot.turn(-90)
    robot.straight(200)
    robot.turn(90)
    robot.straight(200)
    robot.turn(90)
    robot.straight(800)

def goToNext(ev3, robot, sTRight, sTLeft, sColor, DRIVE_SPEED):
    ev3.screen.print("goToNext")
    robot.straight(-50)
    robot.turn(180)
    robot.drive(DRIVE_SPEED, 0)
    while True:
        if sTRight.pressed() or sTLeft.pressed():
            break
    robot.straight(-50)
    robot.turn(-90)
    robot.straight(200)
    robot.turn(90)
    robot.drive(DRIVE_SPEED, 0)
    while True:
        if sColor.color() == Color.BLUE:
            robot.stop()
            break

def move_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=125)
    DRIVE_SPEED = 100
    robot.settings(DRIVE_SPEED, 200, 200, 200)

    driveToWall(ev3, robot, sTRight, sTLeft, sUltra, mSensor, DRIVE_SPEED)
    findBox(ev3, robot, sUltra, mSensor, DRIVE_SPEED)
    moveBoxToCorner(ev3, robot)
    goToNext(ev3, robot, sTRight, sTLeft, sColor, DRIVE_SPEED)