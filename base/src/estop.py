#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
import rospy

class estop():

	def __init__(self): 

		#Set GPIO Pins
		self.estop_pin = 24

		#Setup GPIO
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.estop_pin,GPIO.IN)

		#Setup Parameter
		rospy.set_param('/estop',False)

	def toggle_estop(self):

		if not GPIO.input(self.estop_pin):
			time.sleep(.005)
			if not GPIO.input(self.estop_pin):
				rospy.set_param('/estop',True)
	    
		elif GPIO.input(self.estop_pin):
			time.sleep(.005)
			if GPIO.input(self.estop_pin):
				rospy.set_param('/estop',False)

if __name__ == "__main__":
	rospy.init_node("estop")
	estop = estop()
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
			estop.toggle_estop()
			rate.sleep()


