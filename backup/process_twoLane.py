

import numpy as np
import cv2
import imutils
import math
import os
import time
import config as cf

speed=0
atm= False
count=0
bounding=None
map2 = np.zeros((480,480), dtype=np.uint8)
poly = None
center_pt=None
kernel_size = 5
onelane_detect_l=270
onelane_detect_r=115
# Canny Edge Detector
low_threshold = 200
high_threshold =255
trap_bottom_width = 1  # width of bottom edge of trapezoid, expressed as percentage of image width
trap_top_width = 0.5  # ditto for top edge of trapezoid
trap_height = 1  # height of the trapezoid expressed as percentage of image height
# Hough Transform
rho = 2 # distance resolution in pixels of the Hough grid
theta = 1 * np.pi/180 # angular resolution in radians of the Hough grid
threshold = 35	 # minimum number of votes (intersections in Hough grid cell)
min_line_length = 10 #minimum number of pixels making up a line
max_line_gap = 30	# maximum gap in pixels between connectable line segment

def nothing(x):
	pass
def hough_lines_twolane(blur_gray):
	
	global count,onelane_detect_l, atm
	onelane_detect_l=260
	img = cv2.Canny(blur_gray, low_threshold, high_threshold)
	map2 = np.zeros((500,880), dtype=np.uint8)
	map1 = np.zeros((500,880,3), dtype=np.uint8)
	count += 1
	dataset=[]
	lines1 = cv2.HoughLines(img,2,1*np.pi/180,30)
	if lines1 is not None:
		for line in lines1:
			rho,theta=line[0]
			theta1= theta*180/np.pi  
			cv2.circle(map2, (int(rho)+250,int(theta*180/np.pi + 20)),5,[255,10,25],5)
			#cv2.circle(map2, (250,0),10,[255,10,25],10)	
	else:
		return 0	
	contours,_ = cv2.findContours(map2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #cv2.CHAIN_APPROX_NONE
	
	xmin1=1000
	xmin2=1000
	xmin3 = 700
	wmax = 0
	arr=[]
	temp = np.array([])
	i = 0
	idx = 0
	bounding1 = (0,0,0,0)
	bounding2 = (0,0,0,0)
	for cnt in contours:
		i += 1
		x,y,w,h = cv2.boundingRect(cnt)
		if w>wmax:
			wmax=w
			bounding1=(x,y,w,h)
	
	#if wmax > 45:
	#	cf.chuan_bi_cua = True
	#else:
	#	cf.chuan_bi_cua = False	
	if wmax > 80:
		if bounding1[0] > 220:
			cf.turn_right=True
			cf.turn_left = False
			return 0
		else:
			cf.turn_left=True
			cf.turn_right = False
			return 0
	right_x = []
	right_y = []
	left_x = []
	left_y = []
	for line in lines1:
		rho,theta=line[0]
		rho1=rho+250
		theta1= theta*180/np.pi
		if 90 < theta1 < 180:
				a = np.cos(theta)
				b = np.sin(theta)
				x0 = a*rho
				y0 = b*rho
				x1 = int(x0 + 500*(-b))
				y1 = int(y0 + 500*(a))
				x2 = int(x0 - 500*(-b))
				y2 = int(y0 - 500*(a))
				#cv2.line(img,(x1,y1),(x2,y2),(250,0,255),1)	
				right_x.append(x1)
				right_x.append(x2)
				right_y.append(y1)
				right_y.append(y2)
		elif 0 < theta1 < 90:
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 500*(-b))
			y1 = int(y0 + 500*(a))
			x2 = int(x0 - 500*(-b))
			y2 = int(y0 - 500*(a))
			#cv2.line(img,(x1,y1),(x2,y2),(250,0,255),1)	
			left_x.append(x1)
			left_x.append(x2)
			left_y.append(y1)
			left_y.append(y2)
		
	if len(right_x) > 0 and cf.yes_obstacle == 0:
		right_m,right_b = np.polyfit(right_x, right_y, 1)  # y = m*x + b

		y1 = img.shape[0]
		y2 = int(img.shape[0] * (1 - 0.5))
		right_x1 = int((y1 - right_b) / right_m)
		right_x2 = int((y2 - right_b) / right_m	)
		xt=(right_x2)
		cf.center= (xt-onelane_detect_r)-160
		
		cv2.line(img,(right_x1,y1),(right_x2,y2),(250,0,255),2)

	elif len(left_x) > 0:
		left_m,left_b = np.polyfit(left_x, left_y, 1)  # y = m*x + b

		y1 = img.shape[0]
		y2 = int(img.shape[0] * 0.5)
		left_x1 = int((y1 - left_b) / left_m)
		left_x2 = int((y2 - left_b) / left_m)
		xt=(left_x2)
		if cf.yes_obstacle == 0:
			onelane_detect_l = 230
		cf.center= 160-(onelane_detect_l-xt)
		cv2.line(img,(left_x1,y1),(left_x2,y2),(250,0,255),2)
	
		
	line_img = np.zeros((240 ,320, 3), dtype=np.uint8)
	cf.lane_show2 = map2
	
	line_img[np.where(img[:,:] > 10)]=255
	
	cf.lane_show1= line_img

def two_line(gray):
	line_img = np.zeros((gray.shape[0],gray.shape[1], 3), dtype=np.uint8)  # 3-channel RGB image
	line_img[np.where(gray[:,:] > 100)]=255
	edges = cv2.Canny(line_img, low_threshold, high_threshold)
	line_image = hough_lines(edges)
