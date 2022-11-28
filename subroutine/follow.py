#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
#from ...menu import check_abort


RIGHT_ANGLE = 280


global stop
stop = False

def stop_func(ev3):
    if Button.LEFT in ev3.buttons.pressed():
        return True
    else:
        return False

#box subroutine
def box_subroutine(ev3, sColor, robot):
    print("search box")
    mUS = Motor(Port.C)
    sUS = UltrasonicSensor(Port.S2)
    BLACK = 9
    WHITE = 85
    threshold = 24
    max_distance = 200

    robot.turn(RIGHT_ANGLE)
    robot.drive(200,0)
    while True:
        if sUS.distance() >= max_distance:
            robot.stop()
            break
    
    robot.turn(-RIGHT_ANGLE)
    robot.straight(50)

    robot.drive(200,0)

    while True:
        if sUS.distance() >= max_distance:
            robot.stop()
            break

    robot.turn(-RIGHT_ANGLE)
    robot.straight(50)

    robot.drive(500,0)
    while True:
        if sColor.reflection() > threshold:
            robot.stop()
            #adjust angle so line isnt instantly crossed
            robot.turn(RIGHT_ANGLE - 5)
            robot.straight(50)
            robot.stop()

def gap_subroutine(ev3, color_sensor, mLeft, mRight):
    #drivebase.turn(RIGHT_ANGLE)
    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)
    robot.settings(360, 360, 100, 360)

    speed = 360
    time = 500  
    BLACK = 8
    WHITE = 60
    threshold = 24

    while True:
        robot.reset()
        #if stop_func(ev3):
        #    return

        if color_sensor.reflection() >= threshold:
            return
        else:
            total_angle = 0
            #turn 90 deg left
            robot.drive(0, 200)
            while True:
                if color_sensor.reflection() >= threshold:
                    robot.stop()
                    #line was found, return to line following subroutine
                    return
                elif robot.angle() >= RIGHT_ANGLE:
                    robot.stop()
                    break

            robot.drive(0, -200)
            while True:
                if color_sensor.reflection() >= threshold:
                    robot.stop()
                    robot.turn(20)
                    robot.stop()
                    #line was found, return to line following subroutine
                    return
                elif robot.angle() <= -RIGHT_ANGLE:
                    robot.stop()
                    break

        robot.turn(RIGHT_ANGLE - 10)
        robot.straight(250)
        robot.stop()

def line_follower_controller(ev3, mLeft, mRight, sColor):  
    touchL = TouchSensor(Port.S3)
    touchR = TouchSensor(Port.S4)

    speed = 400
    time = 400       # milliseconds

    # initial measurment
    target_value = 30
    brown = 8
    white = 60


    #used to detect 90° curve or gap 
    error_counter = 0
    #arbitrary number for small enough error
    eps = 1
    print(sColor.reflection())
    robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)
    while True:
        if touchL.pressed() or touchR.pressed():
            box_subroutine(ev3, sColor, robot)
        robot.stop()

        error = target_value - sColor.reflection()

        #determine if section is done
        ev3.screen.print(sColor.reflection())
        #if sColor.color() == Color.BLUE or stop_func(ev3):
        #    print("blue")
        #    return
        #if found_blue(sColor):
        #    return

        if error >= 22:
            error_counter += 1
            if error_counter >= 2:
                print("gap")
                gap_subroutine(ev3, sColor, mLeft, mRight)

        if stop_func(ev3):
            return

        y = error * 8

        #robot.drive(drive_speed, turn_rate)


        if y >= 0:
            #robot.drive(speed, y)
            print("right")
            mLeft.run_time(speed, time - y, wait=True)
            mRight.run_time(speed, time + y, wait=True)

        else:
            print("left")
            mLeft.run_time(speed, time + y, wait=True)
            mRight.run_time(speed, time - y, wait=True)


        robot.drive(200,0)


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
