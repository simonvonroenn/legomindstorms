#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Other Imports
import math

ev3 = EV3Brick()

sections = ["FOLLOW", "SEARCH", "MOVE", "BRIDGE"]
selec = 0

def main_menu():
    global selec

    load_screen()
    while True:
        if Button.DOWN in ev3.buttons.pressed():
            if selec == len(sections) - 1:
                selec = 0
            else:
                selec+=1
            load_screen()
        if Button.UP in ev3.buttons.pressed():
            if selec == 0:
                selec = len(sections) - 1
            else:
                selec-=1
            load_screen()
        while ev3.buttons.pressed():
            wait = 1

def load_screen():
    ev3.screen.clear()

    for i in range(len(sections)):
           ev3.screen.draw_text(20, 20 * i, sections[i])
    
    ev3.screen.draw_circle(5, 10 + 20 * selec, 5, True)
    ev3.screen.draw_text(80, 80, selec)