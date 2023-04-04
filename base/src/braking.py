#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
import rospy
from math import trunc
from geometry_msgs.msg import Twist

class braking():

	def __init__(self): 

		#Set GPIO Pins
		self.brake_enable = 4
		self.brake_a = 17
		self.brake_b = 27
		self.brake_hlfc = 22

		#Setup GPIO
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.brake_enable,GPIO.OUT)
		GPIO.setup(self.brake_a,GPIO.OUT)
		GPIO.setup(self.brake_b,GPIO.OUT)
		GPIO.setup(self.brake_hlfc,GPIO.IN)
		GPIO.output(self.brake_enable, True)
		GPIO.output(self.brake_a, True)
		GPIO.output(self.brake_b, True)

		#Setup Toggle
		self.brake_toggle = False

		#Setup ROS		print(parameters.estop)
		self.image_sub = rospy.Subscriber('/cmd_vel',Twist,self.cmdvelCB)
		self.speed_raw = 0

	def set_brake(self,proportion = 0, shutdown = False):
		if proportion > 50: #100% brake
			a_val, b_val, enable_val = False,False,True

		elif 0 < proportion < 50: #50% brake
			a_val, b_val, enable_val = False,True,True

		elif proportion == 0:
			a_val, b_val, enable_val = True,True,True

		elif shutdown == True:
			a_val, b_val, enable_val = False,False,False

		GPIO.output(self.brake_a, a_val)
		GPIO.output(self.brake_b, b_val)
		GPIO.output(self.brake_enable, enable_val)
			
	def cmdvelCB (self,data):

		if parameters.enable is False:
			speed_scaled = 0

		elif parameters.estop is True:
			speed_scaled = 100
		
		else:
			if data.linear.x >= 0:
				speed_scaled = 0

			elif data.linear.x < 0:
				self.speed_raw = data.linear.x
				speed_scaled = self.speed_raw * -100 / 1.5

			if parameters.estop is False and parameters.enable is True:
				print("Brake set to " + str(trunc(speed_scaled)) + "%")

		self.set_brake(speed_scaled)

class parameters():

	def __init__(self):

		#Set Defaults
		self.estop = False
		self.enable = True

		# Set Toggle
		self.estop_toggle = True
		self.enable_toggle = False
		
	def check(self):
		
		#Check ESTOP
		if rospy.get_param('/estop') is True:
			self.estop = True
			if self.estop_toggle is False:
				print("ESTOP ENABLED")
				print("Brake set to 100%")
				self.estop_toggle = True

		elif rospy.get_param('/estop') is False:
			self.estop = False
			if self.estop_toggle is True:
				print("ESTOP RELEASED")
				self.estop_toggle = False

		#Check ENABLE
		if rospy.get_param('/enable') is True:
			self.enable = True
			if self.enable_toggle is False:
				print("AUTONOMY ENABLED")
				self.enable_toggle = True

		elif rospy.get_param('/enable') is False:
			self.enable = False
			if self.enable_toggle is True:
				print("AUTONOMY DISABLED")
				print("Brake set to 0%")
				self.enable_toggle = False

if __name__ == "__main__":
	rospy.init_node("braking")
	braking = braking()
	parameters = parameters()
	rate = rospy.Rate(10)
	try:
		while not rospy.is_shutdown():
			parameters.check()
			rate.sleep()

	finally:
		braking.set_brake(shutdown=True)