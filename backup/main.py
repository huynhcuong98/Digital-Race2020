#!/usr/bin/env python3
# license removed for brevity
import rospy
import cv2
import numpy as np
import time
import threading
from std_msgs.msg import String, Float32, Int32, Bool, Float32MultiArray
import sensor_msgs.msg
from geometry_msgs.msg import Twist
from itertools import *
import sys
from Camera import get_rgb, get_depth
from process import annotate_image_array, annotate_image_array_far
#from process_phai import annotate_r
#from three_lane import annotate_image_array
import config as cf
import rospkg 
from sign_detection.run_sign import RFB
from drive_car import drive_car_lane
from process_sign import *
from parameter import update_parameter_right_yard


rospy.init_node('run', anonymous=True, disable_signals=True)

cf.time_fps = time.time()
def write_file(self, image):
	
	file = open("./data/data.txt","wb")
	rows, cols = image.shape[:2]
	for row in range(rows):
		file.write('___'+str(row)+'___')
		for col in range(cols):
			info = str(image[row][col][0])+' '
			file.write(info)
		file.write('\n\n')
	file.close()	
	self.count_file = self.count_file + 1
def save_image(image):
	cv2.imwrite("./image/image_"+str(cf.count_img)+".png", image)
	cf.count_img += 1
def set_led(led_stt):
	led_pub = rospy.Publisher('/led_status', Bool, queue_size=1)
	led_pub.publish(led_stt)
def pub_allow_lidar():
	pub = rospy.Publisher('/allow_lidar', Bool, queue_size=1)
	if not cf.pause:
		pub.publish(cf.lidar)
def set_speed(speed):
	if cf.pause or cf.pause2:
		speed = 0
	speed_pub = rospy.Publisher('/set_speed', Float32, queue_size=1)
	speed_pub.publish(speed)

def set_steer(steer):
	steer_pub = rospy.Publisher('/set_angle', Float32, queue_size=1)
	steer_pub.publish(steer)


def set_lcd(text):
	lcd_pub = rospy.Publisher('/lcd_print', String, queue_size=1)
	lcd_pub.publish(text)
def print_lcd(line):
	texts = ["00:0:Speed=", "00:1:Angle=", "00:2:Rec=", "00:3:Pause=", "10:0:Obs="]
	#lane_modes = ["Cla", "Adv", "Line"]
	info = [str(cf.speed), str(cf.steer), str(cf.is_record), str(cf.pause), str(cf.yes_obstacle)]
	text = texts[line]+info[line];
	space = (14 - len(text))*" "
	text +=space
	set_lcd(text)
def get_yaw(data):
	cf.yaw = -data.data
	# re phai --> yaw giam
	# re trai --> yaw tang
	#print("angle yaw", cf.yaw)
def get_obstacle(data):
	if cf.lidar:
		cf.yes_obstacle = data.data
	print("obstacle", cf.yes_obstacle)
		
def get_ss_status(data):
	#set_led(not data.data)
	if data.data is True:
		cf.pause2 = not data.data
	else:
		cf.pause2 = not data.data
	time.sleep(0.1)

def get_bt1_status(data):  #ngoai cung ben phai
	if data.data and not cf.bt1_old:
		cf.pause = not cf.pause
	cf.bt1_old = data.data  
	time.sleep(0.2) 

def get_bt2_status(data):
	if data.data and not cf.bt3_old:
		cf.is_record = not cf.is_record
		cf.frame = 0
	cf.bt3_old = data.data
	time.sleep(0.1)
def get_bt3_status(data):
	if data.data and not cf.bt2_old:
		cf.count=0
	cf.bt2_old = data.data
	time.sleep(0.1)
def clear_all():
	cf.num_turn = 0
	cf.start_yaw = 0
	cf.yaw = 0
	cf.turn_left = False
	cf.turn_right = False
	cf.cua=False
	cf.lane = False
	cf.sign = 'none'
	cf.sign1 = 'none'
	cf.lane_phai=False
	cf.lane_giua = False
	cf.two_lane = False
	cf.vuot_xe = False
	cf.stop_dis=False
	cf.bounding=[]

	cf.speed=0
	cf.speed_max = 22
	cf.steer=0
	cf.yaw = 0.0
	cf.old_yaw = 0.0

	cf.lidar = False
	cf.allow_sign = True
	cf.flag_sign=False
	cf.yes_obstacle = 0

	cf.chuan_bi_cua = False
def get_bt4_status(data):
	if data.data and not cf.bt4_old:
		clear_all()
	cf.bt2_old = data.data
	time.sleep(0.1)
cf.lane_phai=True
cf.lane_giua = False
cf.two_lane = False
cf.vuot_xe = False
cf.switch_mode=True
cf.stop_dis=False
cf.bounding=[]

cf.speed=0
cf.speed_max = 19
cf.steer=0
cf.steer_lidar = 0.0
cf.speed_lidar = 0.0
cf.yaw = 0.0
cf.old_yaw = 0.0

cf.lidar = False
cf.allow_sign = True
cf.flag_sign=False
cf.counter_signs = 0
cf.yes_obstacle = 0

cf.chuan_bi_cua = False

cf.mode = 1
cf.angle=0
cf.angle_old=0
# System Variables
cf.bt1_old =cf.bt2_old = cf.bt3_old = cf.bt4_old = False
cf.is_record = False
cf.out = cv2.VideoWriter('output_01.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (320, 240))
#cf.img = np.zeros((240, 320, 3), np.uint8)
cf.img = np.zeros((240,320,3), np.uint8)
cf.depth =np.zeros((120, 320), np.uint8)
cf.depth1 =np.zeros((240, 320), np.uint8)
cf.depth2 =np.zeros((240, 320), np.uint8)
cf.depth_show=np.zeros((120, 320,3), np.uint8)
cf.sign_show = np.zeros((240, 320,3), np.uint8)
cf.img_show = np.zeros((240, 320,3), np.uint8)
cf.lane_show1 = np.zeros((240, 320,3), np.uint8)
cf.lane_show2 = np.zeros((320,600), dtype=np.uint8)
cf.image_draw = np.zeros((360, 320, 3), np.uint8)
cf.image_record = np.zeros((240,320,3), np.uint8)

cf.count_img = 0
cf.running = True
cf.pause = True
cf.pause2 = True
cf.syn = True
cf.syn1=True
cf.syn2=True
######
cf.center=160
cf.turn_left = False
cf.turn_right = False
cf.cua=False
cf.lane = True

cf.sign = 'none'
cf.sign_flag= False

cf.sign1 = 'none'

cf.have_hori_line = False
cf.hori_dis = 0
cf.allow_sign = True

cf.frame = 0
cf.frame_old = 0
cf.obs = True
cf.start_yaw = 0
def startup():
	cf.start_yaw = cf.yaw
	clear_all()
	time.sleep(0.5)
	print(cf.para)
def lane_pro():
		annotate_image_array(cf.img)	
def main():
	print("Main started!")
	startup()
	while cf.running:
		if cf.syn:
			time_fps = time.time()
			cf.syn = False
			update_parameter_right_yard()
			if cf.lane and not cf.sign_flag:
				try:
					lane_pro()	
				except:
					pass
				drive_car_lane()
			if cf.sign_flag:
				sign()
			#cf.pause = True
			#set_speed(cf.speed)
			#set_steer(cf.steer)
			#print(" goc lai:", cf.steer)
			#pub_allow_lidar()
			#if cf.is_record:
			#	cf.frame += 1
			fps = int(1/(time.time()-time_fps))
			#print("fps", fps)
def show():
	global t_point_fps, time_lcd
	print("Show_thread started !")
	#file = open("./data/data.txt","wb")
	while cf.running:
		img = np.zeros((240, 320, 3), np.uint8)
		img = cf.img.copy()
		img2 = np.zeros((240, 320), np.uint8)
		#img2 = cv2.cvtColor(cf.depth1, cv2.COLOR_GRAY2RGB)
		#img2 = cv2.cvtColor(img2, cv2.COLOR_RGB2BGR)
		#cf.image_record = np.hstack((cf.img, cf.img_show1)) 
		#cf.is_record = True
		#cf.image_record = cv2.putText(cf.image_record, cf.sign1, (150, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA) 
		#cf.image_record = cv2.vconcat([img, cf.img_show1])
		#cf.image_record = cf.img
		#cf.is_record = True
		if cf.is_record:
			print('record')
			cf.out.write(cf.img)
			'''
			if cf.frame != cf.frame_old:
				info = str(cf.frame)+","+str(cf.center+175.0)+"\n"
				byt=info.encode()
				file.write(byt)
				cf.frame_old != cf.frame
				cv2.imwrite("./data5/frame_"+str(cf.frame)+".png", cf.img)	
			'''
		cv2.imshow("sign", cf.sign_show)
		cv2.imshow('lane', cf.lane_show1)
		#cv2.imshow('image2',cf.image_record)
		k = cv2.waitKey(50)
		if k == ord('q'):
			cf.speed = 0
			cf.steer = 0
			#cf.finish = False
			cf.running = False
			#file.close()
			#cf.go = False
			set_speed(cf.speed)
			set_steer(cf.steer)
			cf.out.release()
			rospy.signal_shutdown("shutdown")
		#if k == ord('r'):
		#	cf.obs = False

def listenner():
    rospy.Subscriber("/bt1_status", Bool, get_bt1_status, queue_size=1)
    rospy.Subscriber("/yaw", Float32, get_yaw, queue_size=1)
    rospy.Subscriber("/ss1_status", Bool, get_ss_status, queue_size=1)
    rospy.Subscriber("/bt1_status", Bool, get_bt1_status, queue_size=1)
    rospy.Subscriber("/bt2_status", Bool, get_bt2_status, queue_size=1)
    rospy.Subscriber("/bt3_status", Bool, get_bt3_status, queue_size=1)
    rospy.Subscriber("/bt4_status", Bool, get_bt4_status, queue_size=1)
    rospy.Subscriber("/obstacle", Int32, get_obstacle, queue_size=1)

    rospy.spin()

get_rbg_thread = threading.Thread(name = "get_rbg_thread", target=get_rgb)
get_rbg_thread.start()

get_depth_thread = threading.Thread(name = "get_depth_thread", target=get_depth)
get_depth_thread.start()
time.sleep(0.5)

main_thread = threading.Thread(name= "main_thread", target= main)
main_thread.start()

rfb = RFB()
show_thread = threading.Thread(name= "show information", target= rfb.predict())
show_thread.start()

#sign_thread = threading.Thread(name= "sign_thread", target= predict)
#sign_thread.start()
listenner()
