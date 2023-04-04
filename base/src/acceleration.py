#!/usr/bin/python3

# sudo chmod 666 /dev/spidev0.0
# sudo chmod 666 /dev/spidev0.1
# sudo chmod 666 /dev/gpiomem

import spidev
import time
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist

class acceleration():

	def __init__(self): 

		#Set GPIO Pins
		self.relay_1_pin = 21
		self.relay_2_pin = 20
		self.digipot_pin = 16

		#Setup GPIO
		GPIO.setmode(GPIO.BCM)
		GPIO.setwarnings(False)
		GPIO.setup(self.relay_1_pin,GPIO.OUT)
		GPIO.setup(self.relay_2_pin,GPIO.OUT)
		GPIO.setup(self.digipot_pin,GPIO.OUT)

		# Setup Digipot
		self.spi = spidev.SpiDev()
		self.spi.open(0, 0)
		self.spi.max_speed_hz = 976000

		self.image_sub = rospy.Subscriber('/cmd_vel',Twist,self.cmdvelCB)
		self.speed_raw = 0

		# Set Toggle
		self.switch_toggle = True

	def toggle_switch(self, state):

		if state == "off":
			GPIO.output(self.digipot_pin,GPIO.LOW)
			GPIO.output(self.relay_1_pin,GPIO.LOW)
			GPIO.output(self.relay_2_pin,GPIO.LOW)
			print("Relay set to off")

		elif state == "on":
			GPIO.output(self.digipot_pin,GPIO.HIGH)
			GPIO.output(self.relay_1_pin,GPIO.HIGH)
			GPIO.output(self.relay_2_pin,GPIO.HIGH)
			print("Relay set to on")

	def write_pot(self,input):
		msb = input >> 8
		lsb = input & 0xFF
		self.spi.xfer([msb, lsb])
	
	def cmdvelCB (self,data):
		self.speed_raw = data.linear.x
		speed_byte = self.speed_raw *170
		speed_flipped = int(255-speed_byte)
			
		if speed_flipped >= 255 or parameters.estop is True or parameters.enable is False:

			speed_flipped = 255

			self.write_pot(speed_flipped)

			if self.switch_toggle is True:
				self.toggle_switch("off")
				self.switch_toggle = False
				print("Speed is set to 0 m/s")

		elif speed_flipped < 255:
			self.write_pot(speed_flipped)

			if self.switch_toggle is False:
				self.toggle_switch("on")
				self.switch_toggle = True

			print("Speed is set to " + str(self.speed_raw) + " m/s")

class parameters():

	def __init__(self):

		#Set Defaults
		self.estop = False
		self.enable = True

		# Set Toggle
		self.estop_toggle = False
		self.enable_toggle = True
		
	def check(self):
		
		#Check ESTOP
		if rospy.get_param('/estop') is True:
			self.estop = True
			if self.estop_toggle is False:
				print("ESTOP ENABLED")
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
				self.enable_toggle = False

if __name__ == "__main__":
		rospy.init_node("acceleration")                 
		acceleration = acceleration()
		parameters = parameters()
		rate = rospy.Rate(10)
		try:
			while not rospy.is_shutdown():
				parameters.check()
				rate.sleep()

		finally:
			acceleration.toggle_switch("off")
       
