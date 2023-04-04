#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
import rospy
import math

class encoder():

    def __init__(self): 

		#Set GPIO Pins
        self.encoder_pin = 25

		#Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.encoder_pin,GPIO.IN)

        #Setup Variables
        self.counter = 0
        self.radius = 9  # inches
        self.speed = 0
	
    def read_encoder(self):

        if GPIO.input(self.encoder_pin):
            while GPIO.input(self.encoder_pin):
                time.sleep(0.005)
            self.counter += 1
	    
    def count2speed(self):
        self.speed = 2*math.pi*(self.counter/200)*self.radius
        self.counter = 0
        return self.speed
          
if __name__ == "__main__":
    rospy.init_node("encoder")
    encoder = encoder() 
    rate = rospy.Rate(10)
    previous_time = time.time()
    while not rospy.is_shutdown():
            encoder.read_encoder()
            if time.time() >= previous_time+1:
                print("Speed is " + str(encoder.count2speed()) + "m/s")
    rate.sleep()