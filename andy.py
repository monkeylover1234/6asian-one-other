from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def findRobot(ev3, robot, ultrasonicSensor):
    while ultrasonicSensor.distance() > 500: 
        ev3.screen.print(ultrasonicSensor.distance())
        robot.drive(0,180)

def chargeRobot(ev3, robot, ultrasonicSensor):
    while ultrasonicSensor.distance() <= 500: 
        ev3.screen.print(ultrasonicSensor.distance())
        robot.drive(10000,0)