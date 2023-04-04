#!/usr/bin/python3

import PCANBasic as PCB
import numpy as np
import time
from threading import Lock
from threading import Thread
from tkinter import *
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

can_bus_handle = PCB.PCAN_USBBUS1# this assumes that only one PCAN USB device is connected to the PC
can = PCB.PCANBasic()
can_lock = Lock()
#angle_command_scaled = 0
exit = False

class write_data(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.angle_command_scaled = 0
        self.name = "write_data"
        self.daemon = False
        self.image_sub = rospy.Subscriber('/cmd_vel',Twist,self.cmdvelCB)
        self.start()

    def run(self):
        global can_bus_handle, can, can_lock, exit #angle_command_scaled
        
        len = 6 # CAN message length
        
        x = 0 
        
        system_fault = 0
        message_counter = 0

        # create CAN message
        can_msg = PCB.TPCANMsg()
        can_msg.ID = 0x740
        can_msg.LEN = len
        can_msg.MSGTYPE = PCB.PCAN_MESSAGE_STANDARD


        while not exit:
            veh_spd_scaled = 0# kph
            veh_spd_raw = int((veh_spd_scaled + 0) / 0.015625) # To convert to hex, add offset then  divide by scale factor

            angle_command_raw = int((self.angle_command_scaled + 1638.3) / 0.1) # To convert to hex, add offset then  divide by scale factor
            
            # manipulate bits for each signal to arrange them properly in the message
            can_msg.DATA[0] = (angle_command_raw & 0x7F80) >> 7
            can_msg.DATA[1] = ((angle_command_raw & 0x7F) << 1) | system_fault
            can_msg.DATA[2] =  (veh_spd_raw & 0x7F80) >> 7
            can_msg.DATA[3] = ((veh_spd_raw & 0x7F) << 1)
            can_msg.DATA[4] = message_counter 
            can_msg.DATA[5] = calc_crc(can_msg.DATA, len - 1)

            with can_lock:
                result = can.Write(can_bus_handle, can_msg)
            if result != PCB.PCAN_ERROR_OK:
            # An error occurred, get a text describing the error and show it
            #
                result = can.GetErrorText(result)
                #print(result)

            # update message counter
            message_counter = (message_counter + 1) & 0x0F
         
            # write the msg to the bus every 10 ms
            time.sleep(0.01)
            
    def cmdvelCB (self,data):
        #global angle_command_scaled
        angle_raw = data.angular.z
        self.angle_command_scaled = angle_raw *-2000
        print('cmd: ' + str(self.angle_command_scaled))
        
class read_data(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.name = "read_data"
        self.daemon = False
        self.pos_pub = rospy.Publisher('/steering_angle',Float32,queue_size=10)

        self.start()

    def run(self):
        global can_bus_handle, can, can_lock, exit
        
        error_status = PCB.PCAN_ERROR_OK
        while not exit:
            # Check the receive queue for new messages
            #
            while error_status != PCB.PCAN_ERROR_QRCVEMPTY:
                with can_lock:
                    error_status, can_msg, can_timestamp = can.Read(can_bus_handle)
                if error_status != PCB.PCAN_ERROR_QRCVEMPTY:
                    # Process the received message
                    
                    # !!! YOUR CODE HERE !!!!
                    if can_msg.ID == 0x742:
                        CAN_S_Angle = (can_msg.DATA[0] * 0x100)
                        CAN_S_Angle = (CAN_S_Angle + can_msg.DATA[1])
                        eps_str_angle_raw = int(CAN_S_Angle)

                        if eps_str_angle_raw > 0x8000:
                            eps_str_angle_twos_comp = (eps_str_angle_raw - 65535)
                        else:
                            eps_str_angle_twos_comp = eps_str_angle_raw
            
                        # Apply CAN scale factor
                        eps_str_angle_float = eps_str_angle_twos_comp * 0.0625 * 0.0625
                        print('angle: ' + str(eps_str_angle_float))
                        msg = Float32()
                        msg.data = eps_str_angle_float
                        self.pos_pub.publish(msg)

                else:
                    # An error occurred, get a text describing the error and show it
                    
                    result = can.GetErrorText(error_status)
                    #print(result[1])
            if error_status == PCB.PCAN_ERROR_QRCVEMPTY:
                error_status = PCB.PCAN_ERROR_OK
            else:
                result = can.GetErrorText(error_status)
                #print(result[1])

            # read from the bus every 10 ms
            time.sleep(0.1)
      
def calc_crc(msg, msg_len):
    crc = 0xFF
    crc_poly = 0x1D
    for k in range(0, msg_len):
        crc_cur = msg[k]
        for i in range(0, 8):
            if ((crc & 0x80) == (crc_cur & 0x80)):
                crc = (crc << 1)
            else:
                crc = ((crc << 1) ^ crc_poly) & 0xFF
            crc_cur = (crc_cur << 1)
    
    return (crc ^ 0xFF) & 0xFF

class parameters():

    def __init__(self):

        #Set Defaults
        self.estop = False
        self.enable = True

		# Set Toggle
        self.estop_toggle = False
        self.enable_toggle = True

        #Setup GPIO
        self.ignition_switch = 5
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.ignition_switch,GPIO.OUT)
        GPIO.output(self.ignition_switch,GPIO.HIGH)
		
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
                self.set_ignition(True)

        elif rospy.get_param('/enable') is False:
            self.enable = False
            if self.enable_toggle is True:
                print("AUTONOMY DISABLED")
                self.enable_toggle = False
                self.set_ignition(False)
    
    def set_ignition(self, set):

        self.ignition_switch = 5

        if set is True:
            GPIO.output(self.ignition_switch,GPIO.LOW)
        else:
            GPIO.output(self.ignition_switch,GPIO.HIGH)
    
if __name__ == "__main__":
    rospy.init_node("steering")
    init_can_status = can.Initialize(can_bus_handle, PCB.PCAN_BAUD_500K)
    error_code, error_text = can.GetErrorText(init_can_status)
    can_write_thread = write_data()
    can_read_thread = read_data()
    parameters = parameters()
    rate = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            parameters.check()
            rate.sleep()
    
    finally:
        parameters.set_ignition(False)

        if error_code is not PCB.PCAN_ERROR_OK:
            print("Exiting, CAN init error")
        else:          
            print("Init status CAN: {0}".format(str(error_text)))
            #window.mainloop()
            # can_lock.release()
            exit = True

    
    


            
        
