
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import os
import time
import config as cf
speed=0
atm= False
count=0
bounding=None
map2 = np.zeros((480,640), dtype=np.uint8)
poly = None
center_pt=None
kernel_size = 5
onelane_detect=95
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

lane1=lane2=lane3=[]

def nothing(x):
	pass
def hough_lines(img):
	global count,onelane_detect,atm,speed,lane1,lane2,lane3
	map2 = np.zeros((880,880), dtype=np.uint8)
	count += 1
	dataset=[]
	lines1 = cv2.HoughLines(img,1,1*np.pi/180,30)
	t=time.time()
	for line in lines1:
		rho,theta=line[0]
		theta1= theta*180/np.pi  
		cv2.circle(map2, (int(rho)+250,int(theta*180/np.pi)),5,[255,10,25],5)
		#cv2.circle(map2, (250,0),10,[255,10,25],10)		
	#map2=cv2.dilate(map2,(7,7),iterations = 1)
	#cv2.imshow('img1.jpg',img)
	#cv2.imwrite("img1.jpg",img)
	#cv2.imshow('map',map2)
	#cv2.imwrite("img2.jpg",map2)
	contours,_ = cv2.findContours(map2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	#print(len(contours),'ssss')
	xmin=1000
	xmax=0
	xmid=[]
	arr=[]
	bounding1=[]
	bounding2=[]
	bounding3=[]
	arr1=[]
	boundingBoxes = [cv2.boundingRect(c) for c in contours]
	boundingBoxes = sorted(boundingBoxes,key=lambda b:b[0], reverse=False)# sort left-right
	print(boundingBoxes)
	if len(boundingBoxes)==1:
		print('cua vi co 1 cum')
	elif len(boundingBoxes)==2:
		bounding1=boundingBoxes[0] 
		bounding2=boundingBoxes[1] 
	elif len(boundingBoxes)==3:
		bounding1=boundingBoxes[0] #trai cung
		bounding2=boundingBoxes[2] #giua
		bounding3=(boundingBoxes[1] if boundingBoxes[1][1] <70 else []) #phai cung
	elif len(boundingBoxes)>3:
		bounding1=boundingBoxes[0]
		bounding2=boundingBoxes[-1]
		bounding3=(boundingBoxes[-2] if boundingBoxes[1][1] <70 else [])#phai cung
	arr= bounding1[2]
	if arr >75:
				#print('chuan bi cua')
				speed=14
				#cf.angle_old=cf.angleq
	else:
				if speed < 22:
					speed=speed+ 0.25	
	if bounding1[0]+arr>210:
		atm=True
	if atm :
		atm=False
		print ('cua')
	else:
		print('k cua')
	
	line_draw=[]
	right_x1 = []
	right_y1 = []
	right_x2 = []
	right_y2 = []
	right_x3 = []
	right_y3 = []
	for line in lines1:
			rho,theta=line[0]
			rho1=rho+250
			theta1= theta*180/np.pi	
			if len(bounding1)>0 and rho1>bounding1[0] and rho1<(bounding1[0]+bounding1[2]) and theta1>bounding1[1] and theta1< (bounding1[1]+bounding1[3]):
				a = np.cos(theta)
				b = np.sin(theta)
				x0 = a*rho
				y0 = b*rho
				x1 = int(x0 + 500*(-b))
				y1 = int(y0 + 500*(a))
				x2 = int(x0 - 500*(-b))
				y2 = int(y0 - 500*(a))
				right_x1.append(x1)
				right_x1.append(x2)
				right_y1.append(y1)
				right_y1.append(y2)
			if len(bounding2)>0 and rho1>bounding2[0] and rho1<(bounding2[0]+bounding2[2]) and theta1>bounding2[1] and theta1< (bounding2[1]+bounding2[3]):
				a = np.cos(theta)
				b = np.sin(theta)
				x0 = a*rho
				y0 = b*rho
				x1 = int(x0 + 500*(-b))
				y1 = int(y0 + 500*(a))
				x2 = int(x0 - 500*(-b))
				y2 = int(y0 - 500*(a))
				right_x2.append(x1)
				right_x2.append(x2)
				right_y2.append(y1)
				right_y2.append(y2)
			if len(bounding3)>0 and rho1>bounding3[0] and rho1<(bounding3[0]+bounding3[2]) and theta1>bounding3[1] and theta1< (bounding3[1]+bounding3[3]):
				a = np.cos(theta)
				b = np.sin(theta)
				x0 = a*rho
				y0 = b*rho
				x1 = int(x0 + 500*(-b))
				y1 = int(y0 + 500*(a))
				x2 = int(x0 - 500*(-b))
				y2 = int(y0 - 500*(a))
				right_x3.append(x1)
				right_x3.append(x2)
				right_y3.append(y1)
				right_y3.append(y2)
	lane1=lane2=lane3=[]
	y1 = img.shape[0]
	y2 = 0
	if len(right_x1)>0:
		right_m,right_b = np.polyfit(right_x1, right_y1, 1)  # y = m*x + b		
		right_x11 = int((y1 - right_b) / right_m)
		right_x12 = int((y2 - right_b) / right_m)
		lane1=[(right_x11,y1),(right_x12,y2)]
	if len(right_x2)>0:
		right_m,right_b = np.polyfit(right_x2, right_y2, 1)  # y = m*x + b
		right_x21 = int((y1 - right_b) / right_m)
		right_x22 = int((y2 - right_b) / right_m)
		lane2=[(right_x21,y1),(right_x22,y2)]
	if len(right_x3)>0:
		right_m,right_b = np.polyfit(right_x3, right_y3, 1)  # y = m*x + b
		right_x31 = int((y1 - right_b) / right_m)
		right_x32 = int((y2 - right_b) / right_m)
		lane3=[(right_x31,y1),(right_x32,y2)]
		#cv2.line(img,(right_x1,y1),(right_x2,y2),(250,0,255),2)
	#if a<0.0020 and b<0.6000:
	#	pass#print('khuc cua',count)
	#cv2.imshow('c',img)
	#cv2.imwrite("img3.jpg",img)
	#print(lines1)
	
	line_img = np.zeros((120,320, 3), dtype=np.uint8)  # 3-channel RGB image
	
	#line_img[np.where(img[:,:] > 10)]=255
	return line_img
def filter_colors(image):
	hsv =cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
	lower_white = np.array([0, 0, 230])
	upper_white = np.array([179, 22, 255])
	white_mask = cv2.inRange(hsv, lower_white, upper_white)
	white_image = cv2.bitwise_and(image, image, mask=white_mask)
	#image2 = cv2.addWeighted(white_image, 1., yellow_image, 1., 0.)
	return white_image
def annotate_image_array(image_in):
	global lane1,lane2,lane3
	image = filter_colors(image_in)
	#cv2.imshow('khung2',image)
	#gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blur_gray = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
	edges = cv2.Canny(blur_gray, low_threshold, high_threshold)

	line_image = hough_lines(edges)	
	line_img = np.zeros((120,320, 3), dtype=np.uint8)  # 3-channel RGB image
	line_img = image_in.copy()
	if len(lane1)>0:
		print('co lane1')
		cv2.line(line_img,lane1[0],lane1[1],(0,255,0),3)
	if len(lane2)>0:
		print('co lane2')
		cv2.line(line_img,lane2[0],lane2[1],(0,0,255),3)
	if len(lane3)>0:
		print('co lane3')
		cv2.line(line_img,lane3[0],lane3[1],(255,0,0),3)
	#line_img = np.zeros((120,320, 3), dtype=np.uint8)  # 3-channel RGB image
	cf.img_show1= line_img
	return line_image
'''	 
if __name__ == '__main__':
	cap = cv2.VideoCapture("output_hough.avi")
	while(cap.isOpened()):
		ret,lane_image=cap.read()
		lane_image=lane_image[:,440:]
	#lane_image=cv2.imread('501.png')
		h, w = lane_image.shape[:2]
		#print(w)
		lane_image=cv2.resize(lane_image,(320,240))
		try:
			img=annotate_image_array(lane_image[100:220,:])
			cv2.imshow('khung',lane_image[100:220,:])
			if len(lane1)>0:
				print('co lane1')
				cv2.line(lane_image[100:220,:],lane1[0],lane1[1],(0,255,0),2)
			if len(lane2)>0:
				print('co lane2')
				cv2.line(lane_image[100:220,:],lane2[0],lane2[1],(0,0,255),2)
			if len(lane3)>0:
				print('co lane3')
				cv2.line(lane_image[100:220,:],lane3[0],lane3[1],(255,0,0),2)
			cv2.imshow('img',lane_image)
			#cv2.imwrite("img4.jpg",lane_image)
			cv2.waitKey(0)
		except:
			pass
		#img=annotate_image_array(lane_image[100:220,:])
		#cv2.imshow('khung',lane_image[100:220,:])
		if cv2.waitKey(25) & 0xff == ord('q'):
			
			break    
	cap.release()
	cv2.destroyAllWindows()
'''
 
