#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# Imports
import menu

# Create your objects here.
ev3 = EV3Brick()
mLeft = Motor(Port.A)
mRight = Motor(Port.B)
mSensor = Motor(Port.C)
sColor = ColorSensor(Port.S1)
sUltra = UltrasonicSensor(Port.S2)
sTRight = TouchSensor(Port.S3)
sTLeft = TouchSensor(Port.S4)

# Write your program here.
menu.main_menu(ev3, mLeft, mRight, mSensor, sColor, sUltra, sTRight, sTLeft)