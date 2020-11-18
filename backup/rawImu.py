#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2
import numpy as np
import time
import threading
from std_msgs.msg import String, Float32, Bool
import sensor_msgs.msg
from geometry_msgs.msg import Twist
from itertools import *
import sys
from Camera import get_rgb, get_depth
from process import annotate_image_array
import config as cf
import rospkg 


rospy.init_node('run', anonymous=True, disable_signals=True)
bt1_status = False
bt2_status = False
bt2_old = False
running = False
step = 0

def set_led(led_stt):
    led_pub = rospy.Publisher('/led_status', Bool, queue_size=1)
    led_pub.publish(led_stt)

def set_lcd(text):
    lcd_pub = rospy.Publisher('/lcd_print', String, queue_size=1)
    lcd_pub.publish(text)

def print_lcd(line):
    texts = ["00:0:Speed=", "00:1:Max=", "00:2:Mode=", "00:3:Rec=", "10:0:Angle="]
    lane_modes = ["Cla", "Adv", "Line"]
    info = [str(cf.speed_offset), str(cf.max_addition_speed), lane_modes[cf.lane_mode], str(cf.is_record), str(int(cf.angle))]
    text = texts[line]+info[line];
    space = (14 - len(text))*" "
    text +=space
    set_lcd(text)

def get_imu_angle(data):
	cf.angle=data.data
	print(cf.angle)

def get_bt1_status(data):  #ngoai cung ben phai
	bt1_status = data.data

def get_bt2_status(data):
	if data.data and not bt2_old:
		running = not running
	bt2_old = data.data
	time.sleep(0.1)
			
def main():
	print("Main started!")
	print("nhan button_2 de bat dau.")
	print("nhan button_1 de chot")
	if running:
		file = open("/home/hieu/save_rawImu/data.txt","wb")
		print("*** Begin write ***")
		while running: 
			if bt1_status:
				time.sleep(0.1)
				if bt1_status:
					file.write(str(cf.angle)+" ")
				while bt1_status: None
				print("recorded "+str(step)+"-->"+str(cf.angle))
				step += 5
		step = 0
		file.close()
		print("closed file")

def listenner():
    rospy.Subscriber("/imu_angle", Float32, get_imu_angle, queue_size=1)
    rospy.Subscriber("/bt1_status", Bool, get_bt1_status, queue_size=1)
    rospy.Subscriber("/bt2_status", Bool, get_bt2_status, queue_size=1)
    rospy.spin()

listenner()
main_thread = threading.Thread(name= "main_thread", target= main)
main_thread.start()
