#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import time

from subroutine.utils import straight

def safeTurn(robot, DRIVE_SPEED, degrees):
    sign = 1
    if degrees < 0:
        sign = -1
    robot.drive(DRIVE_SPEED, sign * 26.666) # 40° in total
    time.sleep(1.5)
    robot.stop()
    robot.turn(degrees - sign * 20) # mathematically it should be: degrees - sign * 40

def driveToWall(ev3, robot, sTRight, sTLeft, sUltra, mSensor, DRIVE_SPEED):
    """Dives the robot to the wall.

    First the robots adjusts its distance to the side wall.
    This is necessary because otherwise the robot might drive against the wall.
    After the adjustemnt, the robots drives in a straight line until the touch sensors sense the wall.

    Parameters:
    ev3         --  ev3 brick
    robot       --  drive base
    sTRight     --  right touch sensor
    sTLeft      --  left touch sensor
    sUltra      --  ultra sonic sensor
    mSensor     --  motor which rotates the ultra sonic sensor
    DRIVE_SPEED --  speed at which the robot should drive
    """

    CENTER_DISTANCE = 30
    DRIVE_UNIT = 80
    ADJUSTMENT_PRECISION = 4
    TOLERANCE = 10

    if straight(ev3, robot, 200): return
    # mSensor.run_target(100, -70)
    # while True:
    #     distToCenter = sUltra.distance() - CENTER_DISTANCE
    #     if abs(distToCenter) < TOLERANCE: break
    #     robot.turn(distToCenter / ADJUSTMENT_PRECISION + 5)
    #     straight(ev3, robot, DRIVE_UNIT)
    #     robot.turn( - (distToCenter / ADJUSTMENT_PRECISION + 5))
    
    # mSensor.run_target(100, 0)
    robot.turn(90)
    if straight(ev3, robot, 300): return
    if robot.straight(-50): return
    robot.turn(-90)
    robot.drive(DRIVE_SPEED + 100, 0)
    while True:
        if sTRight.pressed() or sTLeft.pressed():
            break
        if Button.LEFT in ev3.buttons.pressed():
            robot.stop()
            return
    robot.stop()

def findBox(ev3, robot, sUltra, mSensor, DRIVE_SPEED):
    """Finds the box.

    After hitting the side wall, the robot turns around, drives a little bit back and then drives forward and searches for the box.

    Parameters:
    ev3         --  ev3 brick
    robot       --  drive base
    sUltra      --  ultra sonic sensor
    mSensor     --  motor which rotates the ultra sonic sensor
    DRIVE_SPEED --  speed at which the robot should drive
    """

    safeTurn(robot, -DRIVE_SPEED, 180)
    robot.turn(50)
    if straight(ev3, robot, -50): return
    robot.turn(-50)
    mSensor.run_target(100, -70)
    robot.drive(DRIVE_SPEED, 0)
    while True:
        if sUltra.distance() < 500:
            robot.stop()
            robot.straight(200)
            break
        if Button.LEFT in ev3.buttons.pressed():
            robot.stop()
            return
    mSensor.run_target(100, 0)

def moveBoxToCorner(ev3, robot):
    """Moves the box to the corner.

    After finding the box, the robot moves the box to the corner.

    Parameters:
    ev3     --  ev3 brick
    robot   --  drive base
    """

    robot.turn(90) #MAKE SAFE TURN HERE
    if straight(ev3, robot, 1000, 300): return
    if straight(ev3, robot, -50): return
    robot.turn(-90)
    if straight(ev3, robot, 200): return
    robot.turn(90)
    if straight(ev3, robot, 200): return
    if straight(ev3, robot, -20): return
    robot.turn(90)
    if straight(ev3, robot, 1000, 300): return

def goToNext(ev3, robot, sTRight, sTLeft, sColor, DRIVE_SPEED):
    """Drives to the exit.

    After the box has been moved to the corner, the robot drives to the exit of the subroutine.
    Afterwards, the robot begins with the 'bridge' subroutine.

    Parameters:
    ev3         --  ev3 brick
    robot       --  drive base
    sTRight     --  right touch sensor
    sTLeft      --  left touch sensor
    sColor      --  color sensor
    DRIVE_SPEED --  speed at which the robot should drive
    """

    safeTurn(robot, -DRIVE_SPEED, -180)
    robot.drive(DRIVE_SPEED, 0)
    while True:
        if sTRight.pressed() or sTLeft.pressed():
            break
        if Button.LEFT in ev3.buttons.pressed():
            robot.stop()
            return
    if robot.straight(-100): return
    safeTurn(robot, DRIVE_SPEED, -90)
    if robot.straight(200): return
    robot.turn(90)
    robot.drive(DRIVE_SPEED, 0)
    while True:
        if sColor.color() == Color.BLUE:
            robot.stop()
            break
        if Button.LEFT in ev3.buttons.pressed():
            robot.stop()
            return

def move_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
    """The main method of this subroutine.

    This is the main method of this subroutine.
    It coordinates the individual parts of execution.

    Parameters:
    ev3     --  ev3 brick
    mLeft   --  left drive motor
    mRight  --  right drive motor
    mSensor --  motor which rotates the ultra sonic sensor
    sColor  --  color sensor
    sUltra  --  ultra sonic sensor
    sTRight --  right touch sensor
    sTLeft  --  left touch sensor
    """

    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=125)
    DRIVE_SPEED = 150
    robot.settings(DRIVE_SPEED, 200, 200, 200)

    driveToWall(ev3, robot, sTRight, sTLeft, sUltra, mSensor, DRIVE_SPEED)
    findBox(ev3, robot, sUltra, mSensor, DRIVE_SPEED)
    moveBoxToCorner(ev3, robot)
    goToNext(ev3, robot, sTRight, sTLeft, sColor, DRIVE_SPEED)