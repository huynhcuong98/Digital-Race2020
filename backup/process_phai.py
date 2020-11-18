import cv2
import numpy as np
import math
import os
import time
import config as cf
cua=False
count=0
bounding=None

map2 = np.zeros((480,640), dtype=np.uint8)
poly = None
center_pt=None
kernel_size = 5
onelane_detect=115
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

def hough_lines_right(blur_gray):
	global onelane_detect
	img = cv2.Canny(blur_gray, low_threshold, high_threshold)
	
	line_img = np.zeros((240,320, 3), dtype=np.uint8)
	map2 = np.zeros((320,600), dtype=np.uint8)
	dataset=[]
	lines1 = cv2.HoughLines(img,1,1*np.pi/180,30)
	
	if lines1 is not None:
		for line in lines1:
			rho,theta=line[0]
			theta1= theta*180/np.pi  
			cv2.circle(map2, (int(rho)+250,int(theta*180/np.pi)),5,[255,10,25],5)

		contours,_ = cv2.findContours(map2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		xmin = 1000
		dmin = 1000
		arr=[]
		for cnt in contours:
			x,y,w,h = cv2.boundingRect(cnt)
			#d = math.sqrt(x*x+y*y)
			if x<xmin:
				xmin = x
				#dmin = d
				bounding=(x,y,w,h)
				arr.append(w)
		if cf.para['detect turn']:
			if arr[-1] > cf.para['threshold pre turn']:
				print('chuan bi cua')
				#cf.chuan_bi_cua = True
			else:
				cf.chuan_bi_cua = False
			if xmin+arr[-1] > cf.para['threshold turn'] and not cf.sign_flag:#220
				#print("re phai gap !")
				cf.turn_left = True
				return False
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
			if rho1>bounding[0] and rho1<(bounding[0]+bounding[2]) and theta1>y and theta1< (bounding[1]+bounding[3]):
				a = np.cos(theta)
				b = np.sin(theta)
				x0 = a*rho
				y0 = b*rho
				x1 = int(x0 + 500*(-b))
				y1 = int(y0 + 500*(a))
				x2 = int(x0 - 500*(-b))
				y2 = int(y0 - 500*(a))		
				right_x.append(x1)
				right_x.append(x2)
				right_y.append(y1)
				right_y.append(y2)
				#count_line += 1
				#if count_line >= 10:
				#	break

		if len(right_x)>0:
			right_m,right_b = np.polyfit(right_x, right_y, 1)  # y = m*x + b
			#print(a,b,c)
			y1 = img.shape[0]
			y2 = int(img.shape[0] * 0.5)
			right_x1 = int((y1 - right_b) / right_m)
			right_x2 = int((y2 - right_b) / right_m	)
			cv2.line(img,(right_x1,y1),(right_x2,y2),(250,0,255),2)
			xt=right_x2
			cf.center= (xt-onelane_detect)-160

	line_img[np.where(img[:,:] > 10)]=255
	cf.lane_show1 = line_img
	return True
	
def hough_lines_right_far(blur_gray):
	#chuan_bi_cua = False
	#turn_left = False
	img = cv2.Canny(blur_gray, low_threshold-40, high_threshold-40)
	
	line_img = np.zeros((240,320, 3), dtype=np.uint8)
	map2 = np.zeros((320,600), dtype=np.uint8)
	dataset=[]
	lines1 = cv2.HoughLines(img,1,1*np.pi/180,20)
	
	if lines1 is not None:
		for line in lines1:
			rho,theta=line[0]
			theta1= theta*180/np.pi  
			cv2.circle(map2, (int(rho)+250,int(theta*180/np.pi)),3,[255,10,25],2)

		contours,_ = cv2.findContours(map2, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		xmin = 1000
		dmin = 1000
		arr=[]
		for cnt in contours:
			x,y,w,h = cv2.boundingRect(cnt)
			#d = math.sqrt(x*x+y*y)
			if x<xmin:
				xmin = x
				#dmin = d
				bounding=(x,y,w,h)
				arr.append(w)
		x = bounding[0]
		y = bounding[1]
		w = bounding[2]
		h = bounding[3]
		cv2.rectangle(map2, (x,y), (x+w, y+h), (255, 255, 255), 2)
		line_draw=[]
		right_x = []
		right_y = []
		lines1 = lines1.tolist() 
		lines1 = np.array(sorted(lines1, key=lambda a_entry: a_entry[0]))
		#lines1 = lines1[::-1]
		count_line = 0
		for line in lines1:
			rho,theta=line[0]
			rho1=rho+250
			theta1= theta*180/np.pi 
			a=(rho,theta1) 			
			if 90<theta1<145 and rho1>bounding[0] and rho1<(bounding[0]+bounding[2]) and theta1>y and theta1< (bounding[1]+bounding[3]):
				a = np.cos(theta)
				b = np.sin(theta)
				x0 = a*rho
				y0 = b*rho
				x1 = int(x0 + 500*(-b))
				y1 = int(y0 + 500*(a))
				x2 = int(x0 - 500*(-b))
				y2 = int(y0 - 500*(a))		
				right_x.append(x1)
				right_x.append(x2)
				right_y.append(y1)
				right_y.append(y2)
				count_line += 1
				if count_line >= 10:
					break
		if len(right_x)>0:
			right_m,right_b = np.polyfit(right_x, right_y, 1)  # y = m*x + b
			if 57.29*math.atan(right_m) < 20:
				return False
			y1 = img.shape[0]
			y2 = int(img.shape[0] * 0.3)
			right_x1 = int((y1 - right_b) / right_m)
			right_x2 = int((y2 - right_b) / right_m	)
			cv2.line(img,(right_x1,y1),(right_x2,y2),(250,0,255),2)
			xt=right_x2
			cf.center= (xt-40)-160
			#print("error center", cf.center)

	line_img[np.where(img[:,:] > 10)]=255
	cf.lane_show1 = line_img
	#cf.lane_show2 = map2
	return True
