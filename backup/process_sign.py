import numpy as np
import cv2
import math
import time
import config as cf
from process import annotate_image_array_far
from process_cen import hough_lines_cen_far
from drive_car import drive_car_lane

cf.num_right_sign = 0
cf.num_ahead_sign = 0
cf.num_sign = 0
one = True
old_yaw = 0
init_time = 0
error_yaw = np.zeros(3)
dt_yaw = time.time()
one_1 = True
pre_sign = 'none'
num_frame = 0
ahead = True
def sign_counter():
	global pre_sign
	if cf.sign1 != 'none' and pre_sign == 'none':
		cf.num_sign += 1
	pre_sign = cf.sign1
	print("numer sign:", cf.num_sign)
	#print("number turn: ", cf.num_turnleft,  cf.num_turnright,       cf.start_yaw,  cf.yaw)
def pid_yaw(error, p = 1.5, i = 0.1, d = 0.02):
	global error_yaw, dt_yaw
	error_yaw[1:] = error_yaw[0:-1]
	error_yaw[0] = error
	P = error*p
	delta_t = time.time() - dt_yaw
	dt_yaw = time.time()
	D = (error - error_yaw[1]) / delta_t*d
	I = np.sum(error_yaw)*delta_t*i
	angle = P + I + D
	if abs(angle)>60:
		angle = np.sign(angle)*60
	return int(angle)
def clear_pid():
	global error_yaw
	error_yaw = np.zeros(3)
def delay_sign(delay_sec):
	global one, init_time, num_frame, ahead
	if one:
		init_time = time.time()
		cf.num_ahead_sign += 1
		one = False
	elif time.time() - init_time >= delay_sec:
		one = True
		one1 = True
		cf.sign1 = 'none'
		num_frame = 0
		ahead = True
		cf.sign_flag= False
def turn_right():
	global one, old_yaw, num_frame, ahead
	if one:
		old_yaw = cf.yaw
		#cf.num_right_sign += 1
		one = False
	elif abs(old_yaw - cf.yaw) < 85.0:
		cf.speed = 18
		cf.steer = pid_yaw(old_yaw - cf.yaw - 95.0)
		print("yaw:  ", cf.yaw)
	else:
		clear_pid()
		cf.sign1 = 'none'
		one = True
		one1 = True
		ahead = True
		num_frame = 0
		cf.sign_flag = False
def turn_left():
	global one, old_yaw, num_frame, ahead
	if one:
		old_yaw = cf.yaw
		one = False
	elif abs(cf.yaw - old_yaw) < 85.0:
		cf.speed = 18
		cf.steer = pid_yaw(95.0 - cf.yaw + old_yaw)
		print("yaw:  ", cf.yaw)
	else:
		clear_pid()
		cf.sign1 = 'none'
		one = True
		one1 = True
		ahead = True
		num_frame = 0
		cf.sign_flag= False
def sign():
	global one, old_yaw, one_1, num_frame, ahead
	if cf.sign_flag:
		cf.obs = True
	if cf.sign1 == 'right':	
		delay_sign(1)	
		#turn_right()
		
	elif cf.sign1 == 'left':
		delay_sign(1)
		#turn_left()
		
	elif cf.sign1 == 'noleft':
		if ahead:
			blur_gray, cen_img = annotate_image_array_far(cf.img)
			loss = hough_lines_right_far(blur_gray)
			if loss:
				ahead = False
			else:
				delay_sign(1.5)
				driver_car_lane()
		else:
			turn_left()
			
	elif cf.sign1 == 'noright':
		if ahead:
			blur_gray, cen_img = annotate_image_array_far(cf.img)
			loss = hough_lines_right_far(blur_gray)
			if loss:
				ahead = False
			else:
				delay_sign(1.5)
				driver_car_lane()
		else:
			turn_left()
			
	elif cf.sign1 == 'stop':
		cf.pause = True
		cf.sign_flag= False
		cf.sign1 = 'none'
	elif cf.sign1 == 'ahead': # chay lane xa khoang 1.5 giay
		delay_sign(0.5)
	else:
		cf.sign1 = 'none'
		cf.sign_flag= False
			
	
