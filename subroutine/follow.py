#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
#from ...menu import check_abort


RIGHT_ANGLE = 250


global stop
stop = False

def stop_func(ev3):
    if Button.LEFT in ev3.buttons.pressed():
        stop = True

#box subroutine
def box_subroutine(ev3, sColor, robot):
    mUS = Motor(Port.c)
    sUS = UltrasonicSensor(Port.S2)
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2

    #turn USSensor
    mUS.run_time(-RIGHT_ANGLE, 1)
    stop_func(ev3)
    if stop:
        return

    max_distance = 100
    while sUS.distance() < max_distance:
        robot.drive(1000,0)
    stop_func(ev3)
    if stop:
        return
    
    robot.turn(-RIGHT_ANGLE)
    robot.straight(50)

    stop_func(ev3)
    if stop:
        return

    while sUS.distance() < max_distance:
        robot.drive(1000,0)

    robot.turn(-RIGHT_ANGLE)

    while sColor.reflection() <= threshold:
        robot.drive(1000,0)

def gap_subroutine(ev3, color_sensor, drivebase):
    robot.turn(RIGHT_ANGLE)
    BLACK = 9
    WHITE = 85
    threshold = (BLACK + WHITE) / 2

    while True:
        if line_found:
            break
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

def line_follower_controller(ev3, mLeft, mRight, sColor):
    sColor.mode = 'COL-REFLECT'  
    touchL = TouchSensor(Port.S3)
    touchR = TouchSensor(Port.S4)

    speed = 360
    dt = 500       # milliseconds
    stop_action = "coast"

    # initial measurment
    target_value = sColor.value()

    #used to detect 90° curve or gap 
    error_counter = 0
    #arbitrary number for small enough error
    eps = 1

    while True:
        if touchL.pressed() or touchR.pressed():
            box_subroutine(ev3, sColor, robot)

        error = target_value - sColor.value()

        if error > eps:
            error_counter += 1
            #now resets after two consecutive large errors
            if error_counter >= 2:
                gap_subroutine(ev3, sColor, DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104))

        y = 0.5 * error

        if y >= 0:
            mLeft.run_timed(time_sp=dt, speed_sp=speed + y, stop_action=stop_action)
            mRight.run_timed(time_sp=dt, speed_sp=speed - y, stop_action=stop_action)
        else:
            mLeft.run_timed(time_sp=dt, speed_sp=speed - y, stop_action=stop_action)
            mRight.run_timed(time_sp=dt, speed_sp=speed + y, stop_action=stop_action)


def line_follower(ev3, mLeft, mRight, sColor):
    ev3.speaker.beep()
    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)
    touchL = TouchSensor(Port.S3)
    touchR = TouchSensor(Port.S4)



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
        if touchL.pressed() or touchR.pressed():
            box_subroutine(ev3, sColor, robot)
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
