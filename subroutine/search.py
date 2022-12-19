#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


def search_marker(ev3, mLeft, mRight, touchL, touchR, sColor):
    #enters on right side of wall
    #optimal path: up; 90 deg left, short burst, 90 deg left; down; 90 deg right, short burst, 90 deg right;
    #for up and down use drive and continually check for press of button and marker
    #for short burst use straight and two checks for the maerker
    #do for (measure how often)
    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)
    LOOP_NUMBER = 5 #exact value would be Room_length / Robot_size 
    i = 0
    stop = False
    while i < LOOP_NUMBER:
        while not (touchL or touchR):
            if sColot.color() == Color.RED:
                return
        if turn_subroutine(robot, sColor):
            return