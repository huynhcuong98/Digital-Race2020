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
from obstacle_opp import opp

import imutils
import math
point_tracker = Tracker()
rospy.init_node('obs', anonymous=True, disable_signals=True)
scale = 50
fps = 0
time_fps = time.time()
h = int(5.4*50 + 20)
w = int(4*50 + 20)
data_lidar = np.zeros(shape=[2, 360], dtype = np.float)
#image_draw = np.zeros((140, 120, 3), np.uint8)
image_draw = np.zeros((h, w), np.uint8)
image_draw2 = np.zeros((h, w), np.uint8)
image_draw3 = np.zeros((h, w), np.uint8)

obs = []

#########
angle_min = 150 # >0
angle_max = -150 # <0
de2ra = 0.0174533
scale = 50
THRESHOLD_1 = 1

posi_pre = [0,0]
posi_last1 = [0,0]
posi_last2 = [0.0]
allow_scan = False
yes_obstacle = 0
yes_obstacle_old = 0
steer_lidar = 0.0
speed_lidar = 0.0
speed_max = 15

allow_change_lane = True
allow_overtake = True
waiting = False

same = False
opposite = False

running = True 

bphai = 1
btrai = 1
bxa = 4
bsau = 0.2

carX = 25
carY = h - 10
point_0 = (carX, carY)
point_1 = (10, h-20)
point_2 = (10, 10)
point_3 = (w-10, 10)
point_4 = (w-10, h-20)
point_5 = (w//2, (h-10)//2)

bphai = w - 15
btrai = 15
bxa = h - 20
bsau = 0
dt1 = time.time()
dt2 = time.time()
theta1 = []
theta2 = [0,0]
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
	global point_0, point_1, point_2, point_3, point4, point_5
	global posi_last1, posi_last2, dt1, dt2, image_draw2, image_draw3
	global yes_obstacle, yes_obstacle_old, count_predic_f, count_predic_r, direction
	global speed_lidar, steer_lidar, same, opposite
	
	if time.time() - dt1 >= 0.2: 
		cv2.circle(image_draw2, (p[0], p[1]), 3, (255), -1)
		dt1 = time.time()
	
	#direction = 2
	if direction == 0:
		if time.time() - dt2 >= 0.01:
			the = 57.29*math.atan2((point_5[0]-p[0]), (point_5[1]-p[1]))
			predict_direction(the)
			dt2 = time.time()
	elif direction == 2: #same
		crop = image_draw2[point_1[1]-40:point_1[1], point_1[0]+20:point_4[0]]
		image_draw3 = crop
		contours,_ = cv2.findContours(crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.RETR_TREE
		if len(contours) > 0:
			same = True
			print("Move !")
	elif direction == 1: # opposite
		crop = image_draw2[0:point_4[1]-30, point_4[0]//2:point_4[0]]
		image_draw3 = crop
		contours,_ = cv2.findContours(crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.RETR_TREE
		if len(contours) > 0:
			opposite =True	
			print("Move !")	
			
def process(image):
	global data_lidar, angle_min, angle_max, bphai, btrai, bxa, bsau, obs, image_draw, carX, carY
	global point_0, point_1, point_2, point_3, point4, point_5, posi_pre, posi_last
	global x_last, y_last, start
	global yes_obstacle
	x0 = carX
	y0 = carY
	
	phai = bphai
	trai = btrai
	xa = bxa 
	sau = bsau
	
	img = np.zeros((image_draw.shape[0], image_draw.shape[1]), np.uint8)
	
	mask_roi = np.zeros((image.shape[0], image.shape[1]), np.uint8)
	mask_roi2 = np.ones((image.shape[0], image.shape[1]), np.uint8)*255 
	roi = np.array([[(x0-trai, y0+sau), (x0-trai, y0-xa), (x0+phai, y0-xa), (x0+phai, y0+sau)]], dtype=np.int32)
	roi2 = np.array([[(x0-20, y0+10), (x0-20, y0-10), (x0+20, y0-10), (x0+20, y0+10)]], dtype=np.int32)
	mask_roi = cv2.fillPoly(mask_roi, roi, (255, 255, 255))
	mask_roi2 = cv2.fillPoly(mask_roi2, roi2, (0, 0, 0))
	image = cv2.bitwise_and(image, mask_roi) 
	image = cv2.bitwise_and(image, mask_roi2)
	
	contours,_ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.RETR_TREE
	areas = [cv2.contourArea(c) for c in contours]
	contours = sorted(contours, key = cv2.contourArea, reverse=True)[:1]
	if len(contours) > 0:
		x,y,w,h = cv2.boundingRect(contours[0])
		cv2.rectangle(img,(x,y),(x+w,y+h),(255),1)
		posi_pre = [int(x+w/2), int(y+h/2)]
		posi_pre = point_tracker.add(posi_pre)
		cv2.circle(img, (posi_pre[0], posi_pre[1]), 3, (255), -1)
		#print("x,y", obs_position)
	
	cv2.line(img, point_1, point_2, (255), 1)
	cv2.line(img, point_2, point_3, (255), 1)
	cv2.line(img, point_3, point_4, (255), 1)
	cv2.line(img, point_4, point_1, (255), 1)
	
	cv2.circle(img, point_0, 3, (255), -1)
	cv2.circle(img, point_1, 3, (255), -1)
	cv2.circle(img, point_2, 3, (255), -1)
	cv2.circle(img, point_3, 3, (255), -1)
	cv2.circle(img, point_4, 3, (255), -1)
	cv2.circle(img, point_5, 3, (255), -1)
	
	image_draw = img

x_last = 0
y_last = 0
start = 0	
def draw_map():
	global data_lidar, angle_min, angle_max, carX, carY
	global x_last, y_last, start
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
				cv2.circle(image, (x, y), 12, (255), -1)
			x_last = x
			y_last = y	
			#if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
			#	map_pre = np.append(map_pre, [[x, y]], 0)
	if start < 10: 
		start += 1
	else:
		process(image)

def obstacle():
	global angle, speed, yes_obstacle, yes_obstacle_old
	global direction, posi_pre, data_lidar
	global carX, carY, allow_scan, same, opposite
	global dt1, dt2, start
	allow_scan = True
	if allow_scan:
		if not same and not opposite:
			draw_map()
			position(posi_pre)
			
		if opposite:
			yes_obstacle = opp(data_lidar)
			print("obstacle", yes_obstacle)
	else:
		yes_obstacle = 0
		yes_obstacle_old = 0
		dt1 = time.time()
		dt2 = time.time()
		direction = 0
		start = 0
		same = False
		opposite = False
		
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
	global allow_scan
	allow_scan = data.data	      
def pub(array):
	publisher = rospy.Publisher('/obstacle', Int32, queue_size=1)
	publisher.publish(array)
def main():
	global running, fps, time_fps, yes_obstacle
	print("Main started!")
	while running:
		fps = int(1/(time.time()-time_fps))
		time_fps = time.time()
		#print(fps)
		obstacle()
		pub(yes_obstacle)

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
	rospy.Subscriber("/allow_lidar", Bool, get_main, queue_size=1)


main_thread = threading.Thread(name= "main_thread", target= main)
main_thread.start()
show_thread = threading.Thread(name= "show information", target= show)
show_thread.start()

listenner()

