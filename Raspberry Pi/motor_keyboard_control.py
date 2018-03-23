#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time
import atexit
import RPi.GPIO as GPIO
import pygame
import os
from pygame.locals import *


# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr=0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

atexit.register(turnOffMotors)

rearMotor = mh.getMotor(1)
frontMotor = mh.getMotor(2)

# set the speed to start, from 0 (off) to 255 (max speed)
rearMotor.setSpeed(150)
frontMotor.setSpeed(100)

class RCTest(object):

    def __init__(self):
        pygame.init()
        pygame.display.set_mode((100, 100), HWSURFACE | DOUBLEBUF | RESIZABLE)
        self.send_inst = True
        self.steer()

    def steer(self):

        while self.send_inst:
            for event in pygame.event.get():
                if event.type == KEYDOWN:
                    key_input = pygame.key.get_pressed()


                    if key_input[pygame.K_UP] and key_input[pygame.K_RIGHT]:
                        print("Forward Right")
                        rearMotor.run(Adafruit_MotorHAT.FORWARD)
                        frontMotor.run(Adafruit_MotorHAT.FORWARD)
                       

                    elif key_input[pygame.K_UP] and key_input[pygame.K_LEFT]:
                        print("Forward Left")
                        rearMotor.run(Adafruit_MotorHAT.FORWARD)
                        frontMotor.run(Adafruit_MotorHAT.BACKWARD)
                        

                    elif key_input[pygame.K_DOWN] and key_input[pygame.K_RIGHT]:
                        print("Reverse Right")
                        rearMotor.run(Adafruit_MotorHAT.BACKWARD)
                        frontMotor.run(Adafruit_MotorHAT.FORWARD)
                       

                    elif key_input[pygame.K_DOWN] and key_input[pygame.K_LEFT]:
                        print("Reverse Left")
                        rearMotor.run(Adafruit_MotorHAT.BACKWARD)
                        frontMotor.run(Adafruit_MotorHAT.BACKWARD)                       

                    # simple orders
                    elif key_input[pygame.K_UP]:
                        print("Forward")
                        rearMotor.run(Adafruit_MotorHAT.FORWARD)                                    
                               

                    elif key_input[pygame.K_DOWN]:
                        print("Reverse")
                        rearMotor.run(Adafruit_MotorHAT.BACKWARD)                        
                       

                    elif key_input[pygame.K_RIGHT]:
                        print("Right")
                        frontMotor.run(Adafruit_MotorHAT.FORWARD)
                        

                    elif key_input[pygame.K_LEFT]:
                        print("Left")
                        frontMotor.run(Adafruit_MotorHAT.BACKWARD)                       

                    # exit
                    elif key_input[pygame.K_x] or key_input[pygame.K_q]:
                        print ('Exit')
                        self.send_inst = False
                        break

                    #stop the motors
                elif event.type == pygame.KEYUP:
                    rearMotor.run(Adafruit_MotorHAT.RELEASE)                            
                    frontMotor.run(Adafruit_MotorHAT.RELEASE)


if __name__ == '__main__':
    RCTest()
