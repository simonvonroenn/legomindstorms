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

# #positions the robot for driving up the bridge
# def position_upwards(robot):

# used to stay on the bridge by driving with a slight angle and turning when the edge is seen
#
def stay_on(robot, sUltra, sColor, direction, blue):
    ultra_threshold = 130 # placeholder
    distance_threshold = 20 # placeholder
    #continously drive with a slight angle to the right
    robot.drive(100*direction,3*direction)
    BRIDGE_LENGTH = 4000 # placeholder value
    #Continue driving until blue line is found
    while True:
        if blue:
            if sColor.reflection() == Color.BLUE:
                robot.stop()
                break
        #adjust robot angle if it is near the right edge
        if sUltra.distance() > ultra_threshold:
            print(sUltra.distance())
        #if the distance between the last turn is small, the position is at the corner => immediately turn
            robot.stop()
            if robot.distance() * direction < distance_threshold:
                #robot.turn(-70*direction)
                break
            robot.turn(-20*direction)
            robot.reset()
            robot.drive(100*direction,3*direction)
    robot.stop()

def stay_on_V2(robot, sUltra, sColor, direction, section_length, blue):
    ultra_threshold = 150 # placeholder
    speed = 150
    #continously drive with a slight angle to the right
    robot.reset()
    robot.drive(speed*direction,3*direction)
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
            robot.turn(-22*direction)
            prev = robot.distance()          
            robot.drive(speed*direction,3*direction)
    robot.stop()




#To improve the 90 deg right turns, use robot.distance() between turn commands. if distance is too little, imedeadly make a harder turn 
def bridge_main(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft):
    #sUltra.run_angle(200, 45, then=Stop.HOLD, wait=True)
    #125 old axle track
    robot = DriveBase(mLeft, mRight, wheel_diameter=43, axle_track=135)
    robot.settings(200, 200, 200, 200)

    #100cm ramp and 145cm bridge section
    ramp_length = 1000 - 70
    bridge_length = 1450 - 70


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
    stay_on_V2(robot, sUltra, sColor, -1, ramp_length, False)
    #fake right turn, afterward straight
    #return
    robot.turn(-100)
    print("after first")

    #reverse robot again to drive forward on the bridge
    #robot.turn(180)
    #stay on track until next 90 deg turn
    #stay_on(robot, sUltra, sColor, 1,False)
    stay_on_V2(robot, sUltra, sColor, 1, bridge_length, False)
    #robot.straight(-70)
    robot.turn(90)
    #stay on track until blue line
    #stay_on(robot, sUltra, sColor, 1, True)
    stay_on_V2(robot, sUltra, sColor, 1, ramp_length - 40, True)

    #additional code neccessary to find blue line
    #robot is very near to blue line, probably blocked to the left
    # while sColor.reflection() != Color.BLUE:
    #     robot.straight(-10)
    #     robot.turn(-10)
    #     robot.straight(15)

    robot.stop()
    #stop the robot at the end of bridge
    

            

