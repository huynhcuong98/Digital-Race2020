import config as cf
import numpy as np
import time
import threading

cf.error_st = np.zeros(3)
cf.dt_st = time.time()
def PID_steer(error, p= 0.45, i =0.05, d = 0.02):
    cf.error_st[1:] = cf.error_st[0:-1]
    cf.error_st[0] = error
    P = error*p
    delta_t = time.time() - cf.dt_st
    cf.dt_st = time.time()
    D = (error-cf.error_st[1])/delta_t*d
    I = np.sum(cf.error_st)*delta_t*i
    angle = P + I + D
    if abs(angle)>60:
    	angle = np.sign(angle)*60
    return -int(angle)
    

def drive_car_lane():
	max_speed = cf.para['max speed']
	if cf.sign != 'none':
		cf.sign = 'none'
	if cf.turn_right and cf.lane:
		print("cua phai gap")
		cf.steer = -60
		cf.speed = cf.speed_max
		cf.turn_right = False
	elif cf.turn_left and cf.lane:
		print("cua trai gap")
		cf.steer = 60
		cf.speed = max_speed
		cf.turn_left = False
	elif cf.chuan_bi_cua and cf.lane:
		#cf.sign1 = 'none'
		print("chuan bi cua")
		cf.speed = 14
		cf.steer = PID_steer(cf.center)
	
	elif cf.lane:
		if cf.speed < max_speed:
			cf.speed += 0.5
		else:
			cf.speed = max_speed
		cf.speed = max_speed
		cf.steer = PID_steer(cf.center)
	#print("yaw", cf.yawAngle)
	#print("steer, speed", cf.steer, cf.speed)
