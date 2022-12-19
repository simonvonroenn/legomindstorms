#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


def search_marker():
    #enters on right side of wall
    #optimal path: up; 90 deg left, short burst, 90 deg left; down; 90 deg right, short burst, 90 deg right;
    #for up and down use drive and continually check for press of button and marker
    #for short burst use straight and two checks for the maerker
    #do for (measure how often)