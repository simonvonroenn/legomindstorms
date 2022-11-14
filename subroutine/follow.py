#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
#from ...menu import check_abort
import _thread

RIGHT_ANGLE = 250


global stop
stop = False

def stop_func(ev3):
    if Button.LEFT in ev3.buttons.pressed():
        stop = True

#box subroutine
def box_subroutine(ev3, sColor, robot):
    mUS = Motor(Port.c)
    sUS = UltrasonicSensor()

    #turn USSensor
    mUS.run_time(-RIGHT_ANGLE, 1)
    stop_func(ev3)
        if stop:
            break

    threshold = 100
    while sUS.distance() < threshold:
        robot.drive(1000,0)
    
    robot.turn(-RIGHT_ANGLE)
    robot.straight(50)

    while sUS.distance() < threshold:
        robot.drive(1000,0)

    robot.turn(-RIGHT_ANGLE)

    while sColor.reflection() <= threshold:
        robot.drive(1000,0)

def gap_subroutine(ev3, color_sensor, drivebase):
    robot.turn(RIGHT_ANGLE)

    while True:
        stop_func(ev3)
        if stop:
            break

        if sColor.reflection() >= threshold:
            break
        else:

            robot.straight(50)
            total_angle = 0
            line_found = 0
            #turn 90 deg left
            while total_angle <= RIGHT_ANGLE:
                stop_func(ev3)
                if stop:
                    break
                robot.turn(10)
                total_angle += 10
                if sColor.reflection() >= threshold:
                    line_found = 1
                    break
            #turn 180 deg right
            if total_angle >= RIGHT_ANGLE:
                while total_angle > -RIGHT_ANGLE:
                    stop_func(ev3)
                    if stop:
                        break
                    robot.turn(-10)
                    total_angle -= 10
                    if sColor.reflection() >= threshold:
                        line_found = 1
                        break
            robot.turn(RIGHT_ANGLE)



def line_follower(ev3, mLeft, mRight, sColor):
    ev3.speaker.beep()
    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)

    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2

    # Set the drive speed at 100 millimeters per second.
    DRIVE_SPEED = 100

    # Set the gain of the proportional line controller. This means that for every
    # percentage point of light deviating from the threshold, we set the turn
    # rate of the drivebase to 1.2 degrees per second.


    # Start following the line endlessly.
    while True:
        stop_func(ev3)
        if stop:
            break
        #check_abort(ev3, mLeft, mRight, sColor

        if sColor.reflection() >= threshold:
            robot.drive(1000,0)
        else:
            total_angle = 0
            line_found = 0
            #turn 90 deg left
            while total_angle <= RIGHT_ANGLE:
                stop_func(ev3)
                if stop:
                    break
                robot.turn(10)
                total_angle += 10
                if sColor.reflection() >= threshold:
                    line_found = 1
                    break
            #turn 180 deg right
            if total_angle >= RIGHT_ANGLE:
                while total_angle > -RIGHT_ANGLE:
                    stop_func(ev3)
                    if stop:
                        break
                    robot.turn(-10)
                    total_angle -= 10
                    if sColor.reflection() >= threshold:
                        line_found = 1
                        break
            if not line_found:
                gap_subroutine(sColor, robot)
