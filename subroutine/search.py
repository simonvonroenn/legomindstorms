#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import math
import time


foundRed = False
foundWhite = False

def searchSpots(ev3, robot, sColor, ROOM_LENGTH, SPOT_WIDTH, time, TURN_OFFSET):
    """ Search the red and the white spot.

    While the robot drives forward, this method searches for the red and the white spot.
    If the robot finds the first spot, it beeps.
    If the robot find the second spot, it stops and the program returns to menu.

    Parameters:
    robot       --  drive base
    sColor      --  color sensor
    ROOM_LENGTH --  length of the room in millimeters
    SPOT_WIDTH  --  width of the red spot
    time        --  a multiplier that determines how far the robot should drive
                    it is responsible for making the spiral path
    TURN_OFFSET --  an offset that applies only in the first round of the spiral
    """
    
    global foundRed
    global foundWhite

    robot.reset()
    while robot.distance() < (ROOM_LENGTH - time * SPOT_WIDTH - TURN_OFFSET):
        if sColor.color() == Color.RED or sColor.color() == Color.BLUE: # robot sees white as blue
            if not (foundRed or foundWhite): ev3.speaker.beep(500, 200) 
            if sColor.color() == Color.RED: foundRed = True 
            if sColor.color() == Color.BLUE: foundWhite = True 
            if foundRed and foundWhite:
                robot.stop()
                return True
    return False

def turn_left_triangle(robot):
    """ Make a left turn in a triangle shape

    Makes the robot do a left turn, but in a triangle shape.
    This is necessary in the first round of the spiral shape to assure the robot won't touch the wall.

    Parameters:
    robot   --  drive base
    """

    DRIVE_SPEED = 75

    robot.stop()
    robot.drive(-DRIVE_SPEED, -30) # mathematically it should be -26.666
    time.sleep(1.5)
    robot.drive(DRIVE_SPEED, 97.5) # mathematically it should be 86.666
    time.sleep(1.5)
    robot.stop()

def search_main(ev3, mLeft, mRight, sColor):
    """Main function of the search subroutine.

    This is the main function of the search subroutine.
    It creates the spiral shape, the robot should drive, to find the red spot.

    Parameters:
    ev3     --  ev3 brick
    mLeft   --  left drive motor
    mRight  --  right drive motor
    sColor  --  color sensor
    """

    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=135)
    DRIVE_SPEED = 150

    ROOM_LENGTH = 900   # in millimeters
    SPOT_WIDTH = 70     # in millimeters
    TURN_OFFSET = 150
    

    # 1
    robot.drive(50, 0)
    time.sleep(3) # Prevents scanning the blue line. '-150' in searchSpots() to counterbalance this sleep
    robot.drive(DRIVE_SPEED, 0)
    if searchSpots(ev3, robot, sColor, ROOM_LENGTH + 50 - 150, 0, 1, 0): return
    turn_left_triangle(robot)
    # 2
    robot.drive(DRIVE_SPEED, 0)
    if searchSpots(ev3, robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 1, TURN_OFFSET): return
    turn_left_triangle(robot)
    # 3
    robot.drive(DRIVE_SPEED, 0)
    if searchSpots(ev3, robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 1, TURN_OFFSET): return
    turn_left_triangle(robot)
    # 4
    robot.drive(DRIVE_SPEED, 0)
    if searchSpots(ev3, robot, sColor, ROOM_LENGTH, 2 * SPOT_WIDTH, 1, TURN_OFFSET): return
    turn_left_triangle(robot)

    for i in range(2, math.trunc(ROOM_LENGTH/SPOT_WIDTH/2)):
        # 1
        robot.drive(DRIVE_SPEED, 0)
        if searchSpots(ev3, robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 2*i, 0): return
        robot.stop()
        robot.turn(90)
        # 2
        robot.drive(DRIVE_SPEED, 0)
        if searchSpots(ev3, robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 2*i+1, 0): return
        robot.stop()
        robot.turn(90)
        # 3
        robot.drive(DRIVE_SPEED, 0)
        if searchSpots(ev3, robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 2*i+1, 0): return
        robot.stop()
        robot.turn(90)
        # 4
        robot.drive(DRIVE_SPEED, 0)
        if searchSpots(ev3, robot, sColor, ROOM_LENGTH, SPOT_WIDTH, 2*(i+1), 0): return
        robot.stop()
        robot.turn(90)

    # Maybe create alternative searching path, in case the robot didn't find the red spot?