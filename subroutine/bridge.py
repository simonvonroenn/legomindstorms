#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


def stay_on_section(robot, sUltra, sColor, direction, section_length, blue=False, speed=150, turn_angle=-12):
    """Stays on the section of the bridge.
    The robot drives with a slight angle towards the side of the ultra sonic sensor.
    If the ultra sonic sensor detects an edge, the robot turns slightly into the opposite direction.
    Parameters:
    robot       --  the drive base
    sUltra      --  the ultra sonic sensor
    sColor      --  the color sensor
    direction   --  forward (1) or backward (-1)
    blue        --  stop if blue is found, default = False
    speed       --  the speed at which the robot should drive, default = 150
    """
    robot.stop()
    #multiplier to turn harder when on last section
    #blue_mult = 30 if blue else 0
    robot.settings(speed, speed, speed, speed)
    ultra_threshold = 200 # placeholder
    speed = 150
    #continously drive with a slight angle to the right
    robot.reset()
    robot.drive(speed*direction,(3)*direction)
    #Continue driving until blue line is found

    #previous distance value after turn
    prev = 0
    thresh_dist = 200 #cant turn again if distance to previous is not 200
    while robot.distance() * direction < section_length - 20:
        if blue:
            if sColor.reflection() == Color.BLUE:
                robot.stop()
                break
        #adjust robot angle if it is near the right edge
        if sUltra.distance() > ultra_threshold and  (robot.distance() - prev) * direction >= thresh_dist:
            robot.stop()
            robot.turn(turn_angle*direction)
            if blue:
                robot.stop()
                return
            prev = robot.distance()          
            robot.drive(speed*direction,3 *direction)
    robot.stop()


def stay_on_bridge(robot, sUltra, sColor, direction, section_length, speed=150):
    """Stays on the section of the bridge.
    The robot drives with a slight angle towards the side of the ultra sonic sensor.
    If the ultra sonic sensor detects an edge, the robot turns slightly into the opposite direction.
    Parameters:
    robot       --  the drive base
    sUltra      --  the ultra sonic sensor
    sColor      --  the color sensor
    direction   --  forward (1) or backward (-1)
    blue        --  stop if blue is found, default = False
    speed       --  the speed at which the robot should drive, default = 150
    """
    robot.stop()
    #multiplier to turn harder when on last section
    robot.settings(speed, speed, speed, speed)
    ultra_threshold = 150 # placeholder
    speed = 150
    #continously drive with a slight angle to the right
    robot.reset()
    robot.drive(speed*direction,(3 )*direction)
    #Continue driving until blue line is found

    #previous distance value after turn
    prev = 0
    thresh_dist = 200 #cant turn again if distance to previous is not 200
    while robot.distance() * direction < section_length - 20:
        if sColor.reflection() == Color.BLUE:
            robot.stop()
            break
        #adjust robot angle if it is near the right edge
        if sUltra.distance() > ultra_threshold and  (robot.distance() - prev) * direction >= thresh_dist:
            robot.stop()
            robot.turn(turn_angle *direction)
            prev = robot.distance()          
            robot.drive(speed*direction,3 *direction)
    robot.stop()


def bridge_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
    """The entry point for crossing the bridge, drives across the bridge and stops at the blue line.
    The robot drives backwards onto the bridge ramp, turns and drives over middle part and then drives
    down the second ramp until the blue line is found.
    Parameters:
    ev3         --  the ev3 brick
    mLeft       --  the left motor
    mRight      --  the right motor
    mSensor     --  the motor for the ultra sonic sensor
    sColor      --  the color sensor
    sUltra      --  the ultra sonic sensor
    sTRight     --  the right touch sensor
    sTLeft      --  the left touch sensor
    """
    #sUltra.run_angle(200, 45, then=Stop.HOLD, wait=True)
    #125 old axle track
    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=135)
    robot.settings(150, 150, 150, 150)

    #100cm ramp and 145cm bridge section
    ramp_length = 1000 - 70
    bridge_length = 1450 - 80


    #turn robot backwards to drive up the bridge
    robot.straight(-20)
    robot.turn(180)
    #drive up a certain length to guarantee good distance
    robot.straight(-100)
    ev3.screen.clear()
    ev3.screen.draw_text(20, 20, sUltra.distance())
    #stay on track while driving reverse upwards
    #makes problems => try hardcoded variant
    #stay_on(robot, sUltra, sColor, -1, False)
    stay_on_section(robot, sUltra, sColor, -1, ramp_length + 80, speed=50, turn_angle=-17)
    #fake right turn, afterward straight
    #return
    robot.turn(-20)
    robot.straight(30)
    robot.turn(-30)
    robot.straight(30)
    robot.turn(-53)

    print("after first")

    #reverse robot again to drive forward on the bridge
    #robot.turn(180)
    #stay on track until next 90 deg turn
    #stay_on(robot, sUltra, sColor, 1,False)
    stay_on_section(robot, sUltra, sColor, 1, bridge_length - 10)
    #robot.straight(-70)
    robot.turn(90)
    #stay on track until blue line
    #stay_on(robot, sUltra, sColor, 1, True)
    stay_on_section(robot, sUltra, sColor, 1, ramp_length - 300, True, speed=50)
    #robot.straight(ramp_length - 150)
    #robot.turn(-350)
    
    robot.stop()
    robot.reset()
    mSensor.run_angle(150, -220, then=Stop.HOLD, wait=True)

    robot.drive(50, -2)
    while True:
        #somehow blue line is not found
        #ev3.screen.draw_text(20, 20, sUltra.distance())

        if sTLeft.pressed():
            ev3.screen.clear()
            ev3.screen.draw_text(20, 20, sTLeft.pressed())
            robot.stop()
            #robot.straight(-30)
            robot.reset()
            robot.drive(-50,-4)
            while robot.distance() > -100:
                robot.drive(-50,-4)

            robot.turn(-20)
            robot.stop()
            robot.drive(50,0)

        if sTRight.pressed():
            ev3.screen.clear()
            ev3.screen.draw_text(20, 20, sTRight.pressed())
            robot.stop()
            # robot.straight(-30)
            robot.reset()
            robot.drive(-50,4)
            while robot.distance() > -100:
                robot.drive(-50,-4)
            robot.turn(20)
            robot.stop()
            robot.drive(50,0)
        ev3.screen.clear()
        ev3.screen.draw_text(20, 20, sColor.color())
        if sColor.color() == Color.BLUE:
            robot.stop()
            break

    # robot.turn(10)
    ev3.screen.clear()
    ev3.screen.draw_text(20, 20, "done")
    robot.stop()
    robot.settings(150, 150, 150, 150)
    #stop the robot at the end of bridge
    

            

