#!/usr/bin/env python3
# license removed for brevity
import rospy
import cv2
import numpy as np
import time
from std_msgs.msg import String, Float32, Int32, Float32MultiArray, Bool
import sensor_msgs.msg
from geometry_msgs.msg import Twist
from itertools import *
import sys
import config as cf
import rospkg 
import threading
from operator import itemgetter
from tracker import Tracker

import imutils
import math

point_tracker = Tracker()

rospy.init_node('obs', anonymous=True, disable_signals=True)

fps = 0
time_fps = time.time()

data_lidar = np.zeros(shape=[2, 360], dtype = np.float)
#image_draw = np.zeros((140, 120, 3), np.uint8)
image_draw = np.zeros((640, 720, 3), np.uint8)
image_draw2 = np.zeros((280, 460, 3), np.uint8)
field_g = np.zeros((700, 600, 3), np.uint8)

dem = 0
yawOld = 0
yawAngle = 0

scale = 50
THRESHOLD_1 = 1
w = 0.2*scale
d_max = 2*scale
safe_dis1 = 0.5*scale # khoang cach bam duoi (Y)
safe_dis2 = 1.5*scale # Khoang cach phia truoc (Y)
safe_dis3 = 0.2*scale # khoang cach so voi vat can theo truc X
yawOld = 0
carX = 360#60
carY = 320#100
obs = []

#########
angle_min = 100#100 # >0
angle_max = -100#-100 # <0
de2ra = 0.0174533
scale = 50
THRESHOLD_1 = 1
dis_min = [0,0,0]

bphai = 1.5
btrai = 0.15
bxa = 2.2
bsau = 0.3

yes_obstacle = 0
steer_lidar = 0.0
speed_lidar = 0.0
speed_max = 15

yawOld = 0
allow_change_lane = True
allow_overtake = True
waiting = False

counter_signs = 0

running = True 

def sort_obs(ob):
	return ob[1]
def draw_map1():
	global data_lidar, angle_min, angle_max, bphai, btrai, bxa, bsau, obs, image_draw, image_draw2, carX, carY
	global yes_obstacle
	image = np.zeros((image_draw.shape[0], image_draw.shape[1]), np.uint8)
	img = np.zeros((image_draw2.shape[0], image_draw2.shape[1]), np.uint8)
	dist = data_lidar
	for i in range(angle_min, angle_max, -1):
		x = int(carX - scale*dist[0][i]*math.sin(dist[1][i]*de2ra))
		y = int(carY - scale*dist[0][i]*math.cos(dist[1][i]*de2ra))
		if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
			image[y][x] = 255
	image_draw = image
		
def draw_map():
	global data_lidar, angle_min, angle_max, bphai, btrai, bxa, bsau, obs, image_draw, image_draw2, carX, carY
	global yes_obstacle
	x0 = carX
	y0 = carY
	image = np.zeros((image_draw.shape[0], image_draw.shape[1]), np.uint8)
	img = np.zeros((image_draw2.shape[0], image_draw2.shape[1]), np.uint8)
	dist = data_lidar
	for i in range(angle_min, angle_max, -1):
		x = 0
		y = 0
						
		if dist[0][i] < 100: 
			x = int(carX - scale*dist[0][i]*math.sin(dist[1][i]*de2ra))
			y = int(carY - scale*dist[0][i]*math.cos(dist[1][i]*de2ra))
			#cv2.circle(image, (x, y), 5, (255, 255, 255), -1)
			if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
				image[y][x] = 255  

	
	# cat vung roi
	# khoang cach duoc scale 50:1
	# vd: 50 la 1 met ngoai thuc te
	
	
	#phai = 0.8 #met
	#trai = 1
	#xa = 2
	#sau = 1
	
	phai = int(bphai*50)
	trai = int(btrai*50)
	xa = int(bxa*50) 
	sau = int(bsau*50)
	
	mask_roi = np.zeros((image.shape[0], image.shape[1]), np.uint8)
	mask_roi2 = np.ones((image.shape[0], image.shape[1]), np.uint8)*255 
	roi = np.array([[(x0-trai, y0+sau), (x0-trai, y0-xa), (x0+phai, y0-xa), (x0+phai, y0+sau)]], dtype=np.int32)
	roi2 = np.array([[(x0-10, y0+10), (x0-10, y0), (x0+10, y0), (x0+10, y0+10)]], dtype=np.int32)
	mask_roi = cv2.fillPoly(mask_roi, roi, (255, 255, 255))
	mask_roi2 = cv2.fillPoly(mask_roi2, roi2, (0, 0, 0))
	image = cv2.bitwise_and(image, mask_roi) 
	image = cv2.bitwise_and(image, mask_roi2)
	#image_draw = image
	# tien xu ly
	#gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	#gray_image = cv2.GaussianBlur(gray_image, (1, 1), 0)
	# loc ra mau trang (220, 255)
	#mask_white = cv2.inRange(gray_image, 205, 255)
	#image_draw = image
	
	hough = cv2.HoughLines(image,1,1*np.pi/180,5)
	if hough is not None:
		for h in hough:
			rho,theta=h[0]
			rho1=rho+250
			theta1= theta*180/np.pi 
			cv2.circle(img, (int(rho1), int(theta1)), 5, (255,255,255), 5)	
	contours,_ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.RETR_TREE
	xmin=400
	arr=[]
	for cnt in contours:
		x,y,w,h = cv2.boundingRect(cnt)
		if y > 10:
			arr.append([x,y,w,h])
	#print("arr" ,len(arr))
	if len(arr) > 1:
		yes_obstacle = 2
	else:
		yes_obstacle = len(arr)
	
	right_x = []
	right_y = []
	if hough is not None:
		for h in hough:
			rho,theta=h[0]
			rho1=rho+250
			theta1= theta*180/np.pi 
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 500*(-b))
			y1 = int(y0 + 500*(a))
			x2 = int(x0 - 500*(-b))
			y2 = int(y0 - 500*(a))
			cv2.line(image,(x1,y1),(x2,y2),(250,0,255),1)	
			right_x.append(x1)
			right_x.append(x2)
			right_y.append(y1)
			right_y.append(y2)
			
	#print("arr", arr[-1])
	
	image_draw = image
	image_draw2 = img
	
	# findcontours de phan vung cac vat can
	#cnts = cv2.findContours(mask_white, cv2.RETR_EXTERNAL,
	#	cv2.CHAIN_APPROX_SIMPLE)
		
	
angle = 0.0 # do (-60; 60)
speed = 0.0 # (0; cf.speed_max)	
start_dir = 0
loop_dir = 0
dt_dr = time.time()

def obstacle():
	global angle, speed, yes_obstacle, obs, yawOld, yawAngle
	global steer_lidar, speed_lidar, speed_max
	global carX, carY
	draw_map()
	#detect_direction()
	steer_lidar = angle
	speed_lidar = speed				

def get_yaw(data):
	global yawAngle
	yawAngle = data.data

def callback_lidar(data):
	range_angels = np.arange(len(data.ranges))
	global data_lidar
	for i in range(360):
		d = data.ranges[i]
		if d == float("inf"):
			d = 100
		data_lidar[0][i] = d
		if i < 180:
			data_lidar[1][i] = i
		elif i > 180:
			data_lidar[1][i] = i - 360
		else:
			data_lidar[1][i] = 0
def get_signs(data):
	global counter_signs
	counter_signs = data.data	       
def pub(array):
	publisher = rospy.Publisher('/obstacle', Int32, queue_size=1)
	#d = Float32MultiArray(data=array)
	publisher.publish(array)
def main():
	global running, fps, time_fps, yes_obstacle, steer_lidar, speed_lidar
	print("Main started!")
	while running:
		fps = int(1/(time.time()-time_fps))
		time_fps = time.time()
		#print(fps)
		obstacle()
		#pub([yes_obstacle, steer_lidar, speed_lidar])
		pub(yes_obstacle)
		print("yes_obstacle", yes_obstacle)

def show():
	global running, image_draw, image_draw2
	print("Show_thread started !")
	while running:
		cv2.imshow('lidar1', image_draw)
		cv2.imshow('lidar2', image_draw2)
		k = cv2.waitKey(30)
		if k == ord('q'):
			running = False
			rospy.signal_shutdown("shutdown")
def listenner():
	rospy.Subscriber("/yaw", Float32, get_yaw, queue_size=1)
	rospy.Subscriber("/scan", sensor_msgs.msg.LaserScan, callback_lidar, queue_size = 1)
	#rospy.Subscriber("/counting_signs", Int32, get_signs, queus_size=1)


main_thread = threading.Thread(name= "main_thread", target= main)
main_thread.start()
show_thread = threading.Thread(name= "show information", target= show)
show_thread.start()

listenner()

