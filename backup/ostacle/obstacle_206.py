#!/usr/bin/env python3
# license removed for brevity
import rospy
import cv2
import numpy as np
import time
from sklearn.cluster import KMeans
from std_msgs.msg import String, Float32, Int32, Bool, Float32MultiArray, Bool
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
scale = 50
fps = 0
time_fps = time.time()
h = 200
w = 160
data_lidar = np.zeros(shape=[2, 360], dtype = np.float)
#image_draw = np.zeros((140, 120, 3), np.uint8)
image_draw = np.zeros((h, w), np.uint8)
image_draw2 = np.zeros((h, w), np.uint8)
image_draw3 = np.zeros((h, w), np.uint8)
field_g = np.zeros((700, 600, 3), np.uint8)
obs = []

dem = 0
yawOld = 0
yawAngle = 0

scale = 50
w = 0.2*scale
d_max = 2*scale
yawOld = 0
carX = 80#60
carY = 100#100
obs = []
posi_pre = [0,0]
posi_last = [0,0]

#########
angle_min = 120 # >0
angle_max = -120 # <0
de2ra = 0.0174533
scale = 50
bphai = 1
btrai = 0.15
bxa = 3
bsau = 0.3

allow_lidar = False
yes_obstacle = 0
yes_obstacle_old = 0
steer_lidar = 0.0
speed_lidar = 0.0
same = False
opposite = False
running = True 
dt1 = time.time()
dt2 = time.time()
theta1 = []
theta2 = [0,0]
dis = [0, 0]
count_predic_f = 0
count_predic_r = 0
direction = 0 # 2 same, 1 opposite
def predict_direction(the):
	global point_5, dt1, dt2, image_draw2, theta1, theta2
	global yes_obstacle, yes_obstacle_old, count_predic_f, count_predic_r, direction
	theta1.append(the)
	if len(theta1) >= 10:
		mean = np.sum(theta1)/10
		theta2[1] = mean
		if theta2[1] - theta2[0] > 1:
			count_predic_f += 1
			count_predic_r = 0
			if count_predic_f >= 5:
				print("++++++++++++++++++")
				direction = 1
				count_predic_f = 0
		elif theta2[1] - theta2[0] < -1:
			count_predic_r += 1
			count_predic_f = 0
			if count_predic_r >= 5:
				print("------------------")
				direction = 2
				counte_predic_r = 0
		else:	
			count_predic_r = 0
			count_predic_f = 0
			print(" ")
		theta2[0] = theta2[1]
		print("mean angle", mean)
		theta1 = []
def position(p):
	global dis, carX, carY, posi_pre, posi_last
	global dt1, dt2, image_draw2, image_draw3
	global yes_obstacle, yes_obstacle_old, count_predic_f, count_predic_r, direction
	global speed_lidar, steer_lidar, same, opposite
	img = np.zeros((image_draw3.shape[0], image_draw3.shape[1]), np.uint8)

	#direction = 2
	
	if time.time() - dt2 >= 0.2:
		d = math.sqrt((carX-posi_pre[0])**2 + (carY-posi_pre[1])**2)
		if dis[0] == 0:
			dis[0] = d
		else:
			dis[1] = d
			dis_dot = dis[1]-dis[0]
			temp = 0
			if abs(dis_dot) <= 1.5:
				temp = 0.0
			else:
				temp = dis_dot
			
			dis[0] = dis[1]
			print("dis_dot", temp) 
		dt2 = time.time()
		
			
def process(image):
	global data_lidar, angle_min, angle_max, bphai, btrai, bxa, bsau, obs, image_draw, carX, carY
	global posi_pre, posi_last
	global x_last, y_last, start
	global yes_obstacle, steer_lidar, speed_lidar
	x0 = carX
	y0 = carY
	
	phai = int(bphai*50)
	trai = int(btrai*50)
	xa = int(bxa*50) 
	sau = int(bsau*50)
	
	img = np.zeros((image_draw.shape[0], image_draw.shape[1]), np.uint8)
	
	mask_roi = np.zeros((image.shape[0], image.shape[1]), np.uint8)
	mask_roi2 = np.ones((image.shape[0], image.shape[1]), np.uint8)*255 
	roi = np.array([[(x0-trai, y0+sau), (x0-trai, y0-xa), (x0+phai, y0-xa), (x0+phai, y0+sau)]], dtype=np.int32)
	roi2 = np.array([[(x0-10, y0+10), (x0-10, y0-10), (x0+10, y0-10), (x0+10, y0+10)]], dtype=np.int32)
	mask_roi = cv2.fillPoly(mask_roi, roi, (255, 255, 255))
	mask_roi2 = cv2.fillPoly(mask_roi2, roi2, (0, 0, 0))
	image = cv2.bitwise_and(image, mask_roi) 
	image = cv2.bitwise_and(image, mask_roi2)
	
	contours,_ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.RETR_TREE
	areas = [cv2.contourArea(c) for c in contours]
	contours = sorted(contours, key = cv2.contourArea, reverse=True)[:1]
	if len(contours) > 0:
		yes_obstacle = 1
		steer_lidar = 0
		x,y,w,h = cv2.boundingRect(contours[0])
		cv2.rectangle(img,(x,y),(x+w,y+h),(255),1)
		posi_pre = [int(x+w/2), int(y+h/2)]
		posi_pre = point_tracker.add(posi_pre)
		cv2.circle(img, (posi_pre[0], posi_pre[1]), 3, (255), -1)
		
		#d = math.sqrt((x0-posi_pre[0])**2 + (y0-posi_pre[1])**2)
		#print("x,y, dis", (posi_pre[0], posi_pre[1]), d)
		#img = cv2.putText(img, str(d), (150, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255), 2, cv2.LINE_AA) 
		#cv2.line(img,(x0,y0),(posi_pre[0], posi_pre[1]),(255),1)
	image_draw = img

x_last = 0
y_last = 0
start = 0	
def draw_map():
	global data_lidar, angle_min, angle_max, carX, carY
	global x_last, y_last, start, image_draw
	x0 = carX
	y0 = carY
	image = np.zeros((image_draw.shape[0], image_draw.shape[1]), np.uint8)
	
	map1 = np.array([[0, 0]], np.uint8)
	map2 = np.array([[0, 0]], np.uint8)
	dist = data_lidar
	for i in range(angle_min, angle_max, -1):
		x = 0
		y = 0			
		if dist[0][i] < 100: 
			x = int(carX - scale*dist[0][i]*math.sin(dist[1][i]*de2ra))
			y = int(carY - scale*dist[0][i]*math.cos(dist[1][i]*de2ra))
			if abs(x-x_last) <= 7 and abs(y-y_last) <= 7:
				cv2.circle(image, (x, y), 10, (255), -1)
			x_last = x
			y_last = y	
			#if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
			#	map_pre = np.append(map_pre, [[x, y]], 0)
	
	if start < 10:
		start += 1
	else:
		process(image)
		position(posi_pre)

def obstacle():
	global angle, speed, yes_obstacle, yes_obstacle_old, obs, yawOld, yawAngle
	global steer_lidar, speed_lidar, speed_max, direction, posi_pre
	global carX, carY, allow_lidar, same, opposite
	global dt1, dt2, start

	draw_map()
		
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
def get_main(data):
	global allow_lidar
	allow_lidar = data.data	      
def pub(array):
	publisher = rospy.Publisher('/obstacle', Float32MultiArray, queue_size=1)
	d = Float32MultiArray(data=array)
	publisher.publish(d)
def main():
	global running, fps, time_fps, yes_obstacle, steer_lidar, speed_lidar
	print("Main started!")
	while running:
		fps = int(1/(time.time()-time_fps))
		time_fps = time.time()
		#print(fps)
		obstacle()
		pub([yes_obstacle, steer_lidar, speed_lidar])

def show():
	global running, image_draw
	print("Show_thread started !")
	while running:
		cv2.imshow('lidar1', image_draw)
		cv2.imshow('lidar2', image_draw3)
		k = cv2.waitKey(30)
		if k == ord('q'):
			running = False
			rospy.signal_shutdown("shutdown")
def listenner():
	rospy.Subscriber("/yaw", Float32, get_yaw, queue_size=1)
	rospy.Subscriber("/scan", sensor_msgs.msg.LaserScan, callback_lidar, queue_size = 1)
	#rospy.Subscriber("/allow_lidar", Bool, get_main, queue_size=1)


main_thread = threading.Thread(name= "main_thread", target= main)
main_thread.start()
show_thread = threading.Thread(name= "show information", target= show)
show_thread.start()

listenner()

