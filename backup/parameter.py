import cv2
import numpy as np
import config as cf
from process_sign import *

cf.num_turnright = 0
cf.num_turnleft = 0
cf.num_turn = 0
old_num_turn = 0
def turn_counter():
	global old_num_turn
	if abs(cf.yaw - cf.start_yaw) >= 88.0:
		cf.num_turn += 1
		'''
		if cf.yaw > cf.start_yaw:
			cf.num_turnleft += 1
		elif cf.yaw < cf.start_yaw:
			cf.num_turnright += 1
		'''
		cf.start_yaw = cf.yaw
	if old_num_turn != cf.num_turn:
		cf.num_sign = 0
		old_num_turn = cf.num_turn

cf.para = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 22, 'detect lane': True, 'detect turn': True, 'lane right': True, 'lane center': False, 'allow sign': True, 'allow lidar': False, 'overtaking car': False}

stage_0 = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 22, 'detect lane': False, 'detect turn': True, 'lane right': False, 'lane center': False, 'allow sign': False, 'allow lidar': False, 'overtaking car': False}

stage_1 = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 22, 'detect lane': True, 'detect turn': True, 'lane right': True, 'lane center': False, 'allow sign': False, 'allow lidar': False, 'overtaking car': False}

stage_2 = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 22, 'detect lane': True, 'detect turn': False, 'lane right': True, 'lane center': False, 'allow sign': False, 'allow lidar': False, 'overtaking car': False}

stage_3 = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 20, 'detect lane': True, 'detect turn': True, 'lane right': True, 'lane center': False, 'allow sign': True, 'allow lidar': False, 'overtaking car': False}

stage_4 = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 18, 'detect lane': True, 'detect turn': False, 'lane right': True, 'lane center': False, 'allow sign': True, 'allow lidar': False, 'overtaking car': False}

stage_5r = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 25, 'detect lane': True, 'detect turn': True, 'lane right': True, 'lane center': False, 'allow sign': False, 'allow lidar': False, 'overtaking car': False}

stage_6r = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 25, 'detect lane': True, 'detect turn': True, 'lane right': True, 'lane center': False, 'allow sign': False, 'allow lidar': False, 'overtaking car': False}

stage_7r = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 25, 'detect lane': True, 'detect turn': True, 'lane right': True, 'lane center': False, 'allow sign': False, 'allow lidar': False, 'overtaking car': False}

stage_8r = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 25, 'detect lane': True, 'detect turn': True, 'lane right': True, 'lane center': False, 'allow sign': False, 'allow lidar': False, 'overtaking car': False}

route_r = [stage_0, stage_1, stage_2, stage_3, stage_4, stage_5r, stage_6r, stage_7r, stage_8r]
###############################################################################################
###############################################################################################


stage_4ss = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 20, 'detect lane': True, 'detect turn': True, 'lane right': False, 'lane center': True, 'allow sign': False, 'allow lidar': True, 'overtaking car': True}

stage_5s = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 25, 'detect lane': True, 'detect turn': True, 'lane right': False, 'lane center': True, 'allow sign': True, 'allow lidar': True, 'overtaking car': True}

stage_6s1 = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 25, 'detect lane': True, 'detect turn': True, 'lane right': False, 'lane center': True, 'allow sign': False, 'allow lidar': True, 'overtaking car': True}

stage_6s2 = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 25, 'detect lane': True, 'detect turn': False, 'lane right': True, 'lane center': False, 'allow sign': False, 'allow lidar': False, 'overtaking car': False}

stage_7s = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 25, 'detect lane': True, 'detect turn': True, 'lane right': True, 'lane center': False, 'allow sign': True, 'allow lidar': False, 'overtaking car': False}

stage_8s = {'threshold pre turn': 75, 'threshold turn': 220, 'max speed': 25, 'detect lane': True, 'detect turn': True, 'lane right': True, 'lane center': False, 'allow sign': False, 'allow lidar': False, 'overtaking car': False}


route_s = [stage_0, stage_1, stage_2, stage_3, stage_4, stage_5s, stage_6s1, stage_6s2, stage_7s, stage_8s]

def update_parameter_right_yard():
	turn_counter()
	sign_counter()
	print('stage:', cf.num_turn)
	if cf.num_right_sign >= 1:
		cf.para = route_r[cf.num_turn]
	else:
		if cf.num_turn == 4:
			if cf.num_sign == 2:
				route_s[cf.num_turn] = stage_4ss
		else:
			cf.para = route_s[cf.num_turn]
	
	cf.lane_phai = cf.para['lane right']
	cf.lane_giua = cf.para['lane center']
	cf.vuot_xe = cf.para['overtaking car']
	cf.allow_sign = cf.para['allow sign']
	cf.lane = cf.para['detect lane']
	cf.lidar  = cf.para['allow lidar']
		

