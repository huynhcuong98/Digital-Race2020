

import numpy as np
import cv2
import math
import os
import time
import config as cf
#import matplotlib.pyplot as plt
cua=False
count=0
bounding=None
map2 = np.zeros((480,640), dtype=np.uint8)
poly = None
center_pt=None
kernel_size = 5
onelane_detect_l=260
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
max_line_gap = 30	# maximum gap in pixels between connectable line segments

def rhos(line):
	return line[0][0]
def hough_concen(img):
	
	global onelane_detect
	edges = cv2.Canny(img, low_threshold, high_threshold)
	map2 = np.zeros((180,440), dtype=np.uint8)
	dataset=[]
	lines1 = cv2.HoughLines(edges,1,1*np.pi/180,30)
	for line in lines1:
		rho,theta=line[0]
		theta1= theta*180/np.pi  
		cv2.circle(map2, (int(rho)+250,int(theta*180/np.pi)),10,[255,10,25],-1)
	t=time.time()
	#cv2.imshow('imgq',img)
	#cv2.imshow('map',map2)
	contours,_ = cv2.findContours(map2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #cv2.RETR_TREE
	xmin = 440
	arr=[]
	if cf.mode == 1:
		for cnt in contours:
			x,y,w,h = cv2.boundingRect(cnt)
			if x<xmin:
				xmin=x
				bounding=(x,y,w,h)
				arr.append(w)
		#print(arr[-1])
		#print(xmin+arr[-1])
		if arr[-1] > 70:
			#print('chuan bi cua')
			cf.chuan_bi_cua = True
		else:
			cf.chuan_bi_cua = False
		if xmin+arr[-1]>180:
			#print("re phai")
			cf.turn_right=True
	line_draw=[]
	right_x = []
	right_y = []
	lines1 = lines1.tolist() 
	lines1 = np.array(sorted(lines1, key=lambda a_entry: a_entry[0]))
	lines1 = lines1[::-1]
	count_line = 0
	
	for line in lines1:
		rho,theta=line[0]
		rho1=rho+250
		theta1= theta*180/np.pi 
		a=(rho,theta1) 			
		#if rho1>bounding[0] and rho1<(bounding[0]+bounding[2]) and theta1>y and theta1< (bounding[1]+bounding[3]):
		if rho1>100 and 0<theta1<90:
			count_line += 1
			if count_line >= 3:
				break
			a = np.cos(theta)
			b = np.sin(theta)
			x0 = a*rho
			y0 = b*rho
			x1 = int(x0 + 500*(-b))
			y1 = int(y0 + 500*(a))
			x2 = int(x0 - 500*(-b))
			y2 = int(y0 - 500*(a))	
			x3 = 0
			k = rho/np.tan(theta)
			y3 = int(y0 + k*(a))
			#print("k", k, y3)
			#cv2.circle(map2, (x3,y3),5,[255,10,25],5)
			if y3 >= 100:
				#cv2.line(img, (x1, y1), (x2, y2), (255,255,255), 2)
				right_x.append(x1)
				right_x.append(x2)
				right_y.append(y1)
				right_y.append(y2)
	
	if len(right_x)>0:
		right_m,right_b = np.polyfit(right_x, right_y, 1)  # y = m*x + b
		#print(a,b,c)
		y1 = img.shape[0]
		y2 = int(img.shape[0] * 0.5)
		right_x1 = int((y1 - right_b) / right_m)
		right_x2 = int((y2 - right_b) / right_m	)
		cv2.line(edges, (right_x1, y1), (right_x2, y2), (255,255,255), 2)
		xt=right_x2
		cf.center= 160-(onelane_detect_l-xt)
	
	line_img = np.zeros((240,320, 3), dtype=np.uint8)  # 3-channel RGB image
	line_img[np.where(edges[:,:]==255)]=255
	#cf.image_draw = img
	cf.lane_show1 = line_img
	
	#return line_img

