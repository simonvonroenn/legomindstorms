#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# def bridge_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):

#     robot = DriveBase(mLeft, mRight, wheel_diameter=55.5, axle_track=104)

#     # Set the drive speed at 100 millimeters per second.
#     DRIVE_SPEED = 100

#     # Variables for adjusting speed
#     count = 0
#     boost = 0

#     # This variable checks whether the robot passed the first blue line at the start of the parcour.
#     first_blue_line = False

#     # Takes a step back and turns the robot by 180°
#     #robot.straight(-200)
#     #robot.turn(580)

#     # Orientates the Infrared Sensor correctly
#     #mSensor.run_target(20, 90)

#     while not first_blue_line or sColor.color() != Color.BLUE:
#         # Checks if there is an abyss and turns left if so (also acknoledges that first blue line has been passed)
#         if sUltra.distance() > 100:
#             robot.stop()
#             robot.straight(110)
#             robot.turn(280)
#             first_blue_line = True
#         # Drives
#         robot.drive(-1 * (DRIVE_SPEED + boost), 0)
#         #ev3.screen.print(count)
#         # Changes the speed based on the slope of the track
#         if count < 100:
#             count+=1
#         else:
#             #ev3.screen.print(robot.distance())
#             boost = 50 + robot.distance()
#             count = 0
#             robot.reset()

#     robot.stop()

    # Undo Infrared Sensor orientation
    #mSensor.run_target(10, -90)

    # Undo robot rotation
    #robot.turn(580)


def stay_up(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
    ultra_threshold = 100 # placeholder
    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=125)
    robot.settings(200, 200, 200, 200)
    robot.drive(100,5)
    BRIDGE_LENGTH = 4000 # placeholder value
    while robot.distance() <= BRIDGE_LENGTH:
        if sUltra.distance() > ultra_threshold:
            robot.turn(-20)
            robot.drive(100,5)

#To improve the 90 deg right turns, use robot.distance() between turn commands. if distance is too little, imedeadly make a harder turn 
def bridge_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
    ultra_threshold = 100 # placeholder
    distance_threshold = 100 # placeholder
    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=125)
    robot.settings(200, 200, 200, 200)
    #continously drive with a slight angle to the right
    robot.drive(100,5)
    BRIDGE_LENGTH = 4000 # placeholder value
    #Continue driving until blue line is found
    while sColor.color() != Color.BLUE:
        #adjust robot angle if it is near the right edge
        if sUltra.distance() > ultra_threshold:
            #if the distance between the last turn is small, the position is at the corner => immediately turn
            if robot.distance() < distance_threshold:
                robot.turn(-70)
            else:
                robot.turn(-20)
            robot.reset()
            robot.drive(100,5)
    
    #stop the robot at the end of bridge
    robot.stop()

            

