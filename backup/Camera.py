import cv2
import numpy as np
from openni import openni2
from openni import _openni2 as c_api
import config as cf
import time

openni2.initialize() #
dev = openni2.Device.open_any()
rgb_stream = dev.create_color_stream()
rgb_stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, resolutionX=640, resolutionY=480, fps=30))
rgb_stream.start()
depth_stream = dev.create_depth_stream()
depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM, resolutionX = 320, resolutionY = 240, fps = 30))
depth_stream.start()
frames = 0
def get_rgb():
	global frames
	print("Get_rbg started!")
	while cf.running:
		#if not cf.sleep:
		bgr=np.fromstring(rgb_stream.read_frame().get_buffer_as_uint8(),dtype=np.uint8).reshape(480,640,3)
		rgb   = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
		rgb = cv2.resize(rgb,(320, 240))
		cf.img = rgb[:, ::-1, :]
		cf.syn = True
		#if frames < 2:
		#	frames += 1
		#else:
		#	cf.syn2 = True
		#	frames = 0
		#print("Get_rbg stoped")

def get_depth():
	print("Get_depth started!")
	while cf.running:
	     #while not cf.parking and cf.running:
		      #time.sleep(0.05)
		frame = depth_stream.read_frame()
		frame_data = frame.get_buffer_as_uint16()
		img = np.frombuffer(frame_data, dtype=np.uint16)
		img.shape = (240, 320)
		img = cv2.flip(img , 1)
		img3 = (img*1.0/2**8).astype(np.uint8)
		#cf.depth1 = img3.copy()
		img = img[:, :]
		#print(cf.depth.shape)
		#cv2.imshow('img',img)
		img2 = img.copy()
		img2 = (img2*1.0/2**8).astype(np.uint8)
		#img2 = 255 - img2
		img2[np.where(img2[:,:]==0)]=255
		cf.depth1 = img2
		
		'''
		if cf.flag_sign:
			#print(cf.sign_turn_right)
			cf.avg = img2[int(cf.bounding[1]+(cf.bounding[3]//2)),int(cf.bounding[0]+(cf.bounding[2]//2))]
			dis = img2[int(cf.bounding[1]+(cf.bounding[3]//2)),int(cf.bounding[0]+(cf.bounding[2]//2))]
			print("distance", dis)
			if dis <=50:
				cf.pause=True
			#if cf.switch_mode:
			#	cf.switch_mode=False
			#	cf.angle_old= cf.angle
			#	cf.count_sign+=1
			#if cf.count_sign ==1 :
			#	cf.sign_turn_right=True 
			#elif cf.count_sign ==2:
			#	cf.sign_turn_right=True
			#	cf.mode =1 
			#elif cf.count_sign==3:
				#cf.stop_dis=True
			#if cf.count_sign >3:
			#	cf.count_sign=3
		
	     	#adap = cv2.adaptiveThreshold(img2, 255, cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,11,-2)
		
		ret, img2=cv2.threshold(img2,115,255,cv2.THRESH_BINARY_INV)#threshold remove background 
		cf.depth2= img2
		line_one=np.zeros((120,8))
		line_one1=np.zeros((10,320))
		img2=np.concatenate((line_one1,img2),axis=0)
		img2=np.concatenate((img2[:,8:],line_one),axis=1)
		img2=np.stack((img2,)*3,axis=-1).astype(np.uint8)
		a=cv2.bitwise_and(cf.img[0:120,:],img2)
		a=a.astype(np.uint8)
		cf.depth_show=a[0:85,:,:]
		'''
		cf.syn1=True
		
