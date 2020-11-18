import numpy as np
import cv2
import math
import os
import time
import config as cf

# 0: TURN LEFT
# 1: TURN RIGHT
# 2: STRAIGHT AHEAD
array_turnLeft = []
array_turnRight = []
black_box = []
route = []
pre_map_yaw = 0
def a():
	global pre_map_yaw
	if abs(pre_map_yaw - cf.yaw) >= 90.0:
		if cf.yaw > pre_map_yaw:
			black_box.append(0)
		elif cf.yaw < pre_map_yaw:
			black_box.append(1)
		pre_map_yaw = cf.yaw
def map_pro():
	global map_yaw
	
	
