

import numpy as np
import cv2
import math
import os
import time
import config as cf
from process_cen import hough_lines_cen
#from process_lef import lef_line
from process_twoLane import hough_lines_twolane
from process_phai import hough_lines_right, hough_lines_right_far
from pro_concen import hough_concen

kernel_size = 3
def demo_far(img):
	dai  = img.shape[1]
	rong = img.shape[0]
	cen_img = np.zeros((rong, dai), dtype=np.uint8)
	cen_img = img.copy()
	draw = np.ones((rong, dai), dtype=np.uint8)*255
	mask_roi_cen = np.ones((rong, dai), dtype=np.uint8)*255
	cnts,_ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #cv2.CHAIN_APPROX_NONE
	distance = 240
	if cnts is not None:
		for cnt in cnts:
			x,y,w,h = cv2.boundingRect(cnt)
			d = math.sqrt(w**2 + h**2)
			# gan bien 20, 5, duong cheo >70
			
			if x < 20 or dai-x-w < 20 or d > 30:
				roi = np.array([[(x, y+h), (x, y), (x+w, y), (x+w, y+h)]], dtype=np.int32)
				mask_roi_cen = cv2.fillPoly(mask_roi_cen, roi, (0))
				cen_img = cv2.bitwise_and(img, mask_roi_cen)
	return cen_img
def demo(img):
	dai  = img.shape[1]
	rong = img.shape[0]
	cen_img = np.zeros((rong, dai), dtype=np.uint8)
	two_img = np.zeros((rong, dai), dtype=np.uint8)
	cen_img = img.copy()
	two_img = img.copy()
	draw = np.ones((rong, dai), dtype=np.uint8)*255
	mask_roi_cen = np.ones((rong, dai), dtype=np.uint8)*255
	mask_roi_two = np.ones((rong, dai), dtype=np.uint8)*255
	cnts,_ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #cv2.CHAIN_APPROX_NONE
	distance = 240
	if cnts is not None:
		for cnt in cnts:
			x,y,w,h = cv2.boundingRect(cnt)
			d = math.sqrt(w**2 + h**2)
			# gan bien 20, 5, duong cheo >70
			
			if x < 20 or dai-x-w < 20 or y < 5 or d > 70:
			
				roi = np.array([[(x, y+h), (x, y), (x+w, y), (x+w, y+h)]], dtype=np.int32)
				mask_roi_cen = cv2.fillPoly(mask_roi_cen, roi, (0))
				cen_img = cv2.bitwise_and(img, mask_roi_cen)
			
			if d < 50:
				roi = np.array([[(x, y+h), (x, y), (x+w, y), (x+w, y+h)]], dtype=np.int32)
				mask_roi_two = cv2.fillPoly(mask_roi_two, roi, (0))
				two_img = cv2.bitwise_and(img, mask_roi_two)
	
	return cen_img, two_img
def filter_colors(image):
	hsv =cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
	lower_white = np.array([0, 0, 230])
	upper_white = np.array([179, 22, 255])
	white_mask = cv2.inRange(hsv, lower_white, upper_white)
	white_image = cv2.bitwise_and(image, image, mask=white_mask)
	#image2 = cv2.addWeighted(white_image, 1., yellow_image, 1., 0.)
	return white_image
def annotate_image_array(image_in):
	image = image_in[100:220,:]
	image = filter_colors(image)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
	loss_lane = False
	if cf.vuot_xe:
		cen_img, two_img = demo(blur_gray)
		loss_lane = hough_lines_cen(cen_img)
		print("loss lane  ", loss_lane)
		if loss_lane:
			hough_lines_twolane(two_img)	
	elif cf.lane_phai:
		go_ahead = hough_lines_right(blur_gray)
	elif cf.lane_giua:
		hough_concen(blur_gray)
def annotate_image_array_far(image_in):
	image = image_in[20:120,:]
	cen_img = np.zeros_like(image)
	image = filter_colors(image)
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)
	if cf.vuot_xe:
		cen_img = demo_far(blur_gray)
	'''
	if cf.sign == 'noright':
		if cf.vuot_xe:
			loss = hough_lines_cen_far(cen_img)
	hough_lines_right_far(blur_gray)
	'''
	return blur_gray, cen_img
		
	

	
	
	

