#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
import rospy

class enable():

	def __init__(self): 

		#Set GPIO Pins
		self.enable_pin = 23

		#Setup GPIO
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.enable_pin,GPIO.IN) 

		#Setup Parameter
		rospy.set_param('/enable',False)

	def toggle_enable(self):

		if GPIO.input(self.enable_pin):
			time.sleep(.005)
			if GPIO.input(self.enable_pin):
				rospy.set_param('/enable',True)
	    
		elif not GPIO.input(self.enable_pin):
			time.sleep(.005)
			if not GPIO.input(self.enable_pin):
				rospy.set_param('/enable',False)

if __name__ == "__main__":
	rospy.init_node("enable")
	enable = enable()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
			enable.toggle_enable()
			rate.sleep()


