#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


RIGHT_ANGLE = 90 #deg

global stop
stop = False

def stop_func(ev3):
    if Button.LEFT in ev3.buttons.pressed():
        return True
    else:
        return False

#box subroutine
def box_subroutine(ev3, sColor, robot):
    """Drives around the box and makes sure to continue following the line 
    Parameters:
    ev3         --  ev3
    robot       --  the drive base
    sColor      --  the color sensor
    """
    print("search box")
    robot.straight(-40)
    robot.turn(-RIGHT_ANGLE)
    robot.straight(200)
    robot.turn(RIGHT_ANGLE)
    robot.straight(350)
    robot.turn(60)# - 45)
    robot.straight(200)
    robot.turn(-60)
    robot.straight(-50)
    #robot.turn(-RIGHT_ANGLE)
    robot.reset()
    robot.drive(100,0)
    while True:
        if sColor.color() == Color.BLUE or robot.distance() > 20:
            robot.stop()
            break

    #robot.turn(-45)

def gap_subroutine(ev3, color_sensor, mLeft, mRight):
    """Handles refinding the line and gaps in the track.
    Tries to find the line again using increments of left and right turns.
    If line was not found, drive forward and try again
    Parameters:
    ev3         --  ev3
    mLeft       --  left motor
    mRight      --  right motor 
    color_sensor      --  the color sensor
    """
    #drivebase settings
    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=125)
    robot.settings(200, 200, 200, 200)
   
    threshold_l = 23
    threshold_r = 15

    #normal turns
    angles = [20, 70]
    robot.straight(10)
    for i in angles:
        if color_sensor.reflection() >= threshold_l:
            return
        robot.reset()
        robot.drive(0, 100)
        while True:
            #ev3.screen.print(color_sensor.reflection())
            if color_sensor.reflection() >= threshold_l:
                robot.stop()
                #line was found, return to line following subroutine
                return i * 2
            elif robot.angle() >= i:
                ev3.screen.print(robot.angle())
                robot.stop()
                break

        robot.drive(0, -100)
        while True:
            #ev3.screen.print(color_sensor.reflection())
            ev3.screen.print(color_sensor.reflection())
            if color_sensor.reflection() >= threshold_r:
                robot.stop()
                #line was found, return to line following subroutine
                return -i * 2
            elif robot.angle() <= -i:
                #ev3.screen.print(robot.angle())
                robot.stop()
                break
        robot.turn(i)



    #hard right turn
    for i in [1,1,1,1,1,1]:
        robot.drive(0, -100)
        while True:
            #ev3.screen.print(color_sensor.reflection())
            ev3.screen.print(color_sensor.reflection())
            if color_sensor.reflection() >= threshold_r:
                line_found = True
                robot.stop()
                #line was found, return to line following subroutine
                return 0
            elif robot.angle() <= -20:
                #ev3.screen.print(robot.angle())
                robot.reset()
                robot.stop()
                break
    robot.drive(0,100)
    while robot.angle() <= 120:
        if color_sensor.reflection() >= threshold_l:
            robot.stop()
            return 0
    robot.stop()


    #drive straight to go over gap
    robot.reset()
    robot.drive(100,0)
    while robot.distance() < 150:
        if color_sensor.reflection() >= threshold_l:
            robot.stop()
            break
    return 0
         
        


   

def line_follower_controller(ev3, mLeft, mRight, sColor, sTRight, sTLeft):
    """Handles following the line.
    Main part of line following. Tries to stay on the line by detecting curves and driving accordingly.
    Detects if line is lost.
    Parameters:
    ev3         --  ev3
    mLeft       --  left motor
    mRight      --  right motor 
    sColor      --  the color sensor
    sTRight     --  right touch sensor
    sTLeft      --  left touch sensor
    """
    # initial measurment
    target_value = 30#35
    brown = 8
    white = 60


    #used to detect 90° curve or gap 
    error_counter = 0
    #arbitrary number for small enough error
    eps = 1
    print(sColor.reflection())
    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=125)
    turn_rate = 0
    while True:
        robot.stop()

        error = target_value - sColor.reflection()

        if stop_func(ev3):
            return

        robot.drive(200, turn_rate)
        while True:
            if sTLeft.pressed() or sTRight.pressed():
                box_subroutine(ev3, sColor, robot)
                return
            if error > 17:
                robot.stop()
                break
            error = target_value - sColor.reflection()
        new_turn = gap_subroutine(ev3, sColor, mLeft, mRight)
        turn_rate = turn_rate + new_turn if (new_turn > 0 and (turn_rate * new_turn >= 0))  else new_turn


def found_blue(sColor):
    if sColor.color() == Color.BLUE:
        print("blue")
        return True
    else:
        return False

def program_aborted():
    if Button.CENTER in ev3.buttons.pressed():
        return True
    else:
        return False

def line_follower(ev3, mLeft, mRight, sColor):
    """Deprecated version of line-following
    Parameters:
    ev3         --  ev3
    mLeft       --  left motor
    mRight      --  right motor 
    color_sensor      --  the color sensor
    """
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
                gap_subroutine(sColor, mLeft, mRight)
