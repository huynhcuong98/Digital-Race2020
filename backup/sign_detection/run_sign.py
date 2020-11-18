"""
This code uses the pytorch model to detect faces from live video or camera.
"""
import config as cf
import argparse
import sys
import cv2
import numpy as np
import time
from sign_detection.SVM import predict_recblue, predict_cirred
from sign_detection.vision.ssd.config.fd_config import define_img_size


parser = argparse.ArgumentParser(
    description='detect_video')
#source_vid= 'data_vid_thor.avi', 'output_4.avi', 'clip.mp4', 'clip1.mp4', 'clip2.mp4'
#name_vid = source_vid[0]
parser.add_argument('--net_type', default="RFB", type=str,
                    help='The network architecture ,optional: RFB (higher precision) or slim (faster)')
parser.add_argument('--input_size', default=320, type=int,
                    help='define network input size,default optional value 128/160/320/480/640/1280')
parser.add_argument('--threshold', default=0.99, type=float,
                    help='score threshold')
parser.add_argument('--candidate_size', default=1000, type=int,
                    help='nms candidate size')
parser.add_argument('--path', default="imgs", type=str,
                    help='imgs dir')
parser.add_argument('--test_device', default="cuda:0", type=str,
                    help='cuda:0 or cpu')
#parser.add_argument('--video_path', default=f"vid/{name_vid}", type=str,
#                    help='path of video')
args = parser.parse_args()

input_img_size = args.input_size
define_img_size(input_img_size)  # must put define_img_size() before 'import create_mb_tiny_fd, create_mb_tiny_fd_predictor'

from sign_detection.vision.ssd.mb_tiny_fd import create_mb_tiny_fd, create_mb_tiny_fd_predictor
from sign_detection.vision.ssd.mb_tiny_RFB_fd import create_Mb_Tiny_RFB_fd, create_Mb_Tiny_RFB_fd_predictor
from sign_detection.vision.utils.misc import Timer



class RFB(object):
	def __init__(self):
		label_path = "sign_detection/models/voc-model-labels.txt"

		net_type = args.net_type

		class_names = [name.strip() for name in open(label_path).readlines()]
		num_classes = len(class_names)
		test_device = args.test_device

		candidate_size = args.candidate_size
		threshold = args.threshold


		model_path = "sign_detection/models/pretrained/RFB-Epoch-4645-Loss-0.03478194429324223.pth"
		# model_path = "models/pretrained/version-RFB-640.pth"
		net = create_Mb_Tiny_RFB_fd(len(class_names), is_test=True, device=test_device)
		self.predictor = create_Mb_Tiny_RFB_fd_predictor(net, candidate_size=candidate_size, device=test_device)

		net.load(model_path)

		timer = Timer()
		sum = 0
		k=0
		predict_array = []
		print("start sign detect---------")

	def predict(self):
		global k, predict_array
		count=0
		while cf.running:
			if cf.syn:
				count+=1
				if count % 2 == 1: 
					count=0
					count2=0
					#print("fps", cf.fps)
					cf.syn = False
					image = cf.img.copy()
					depth_image = cf.depth1.copy()
					image[:,0:180]=(0,0,0)
					#image = cv2.cvtColor(orig_image, cv2.COLOR_BGR2RGB)
					timer.start()
					boxx = 0
					boxy = 0
					try:
						if cf.sign_flag== False:
							boxes, labels, probs = self.predictor.predict(image, candidate_size / 2, threshold)
							interval = timer.end()
							#print("flag", cf.sign_flag)
							#print('Time: {:.6f}s, Detect Objects: {:d}.'.format(interval, labels.size(0)))
							for i in range(boxes.size(0)):
								box = boxes[i, :]
								label = f" {probs[i]:.2f}"
								x,y,x2,y2 = int(box[0].numpy()), int(box[1].numpy()), int(box[2].numpy()), int(box[3].numpy())
								roi = image[y:y2,x:x2]
								boxy = int((box[1]-10+box[3])/2)
								boxx = int((box[0]+8+box[2])/2)
								dis = depth_image[boxy,boxx]
								#print("dis", dis)
								# if dis <= 50 and not cf.sign_flag and cf.allow_sign:
								if dis <= 50 and cf.allow_sign:
									pre = predict_cirred(roi)
									tag = labels.numpy()
									cv2.rectangle(image, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
									cv2.putText(image,pre,(box[0],box[3]-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,211,255),2)
									#print("x, y:", boxx, boxy)
									#print("dx, yy:", abs(x-x2), abs(y-y2))
									print("predict: ", pre)
									#if pre != 'none':
										#cf.sign = pre
									cf.sign1 = pre
									#else:
									#	print("casfaslfafafa")
									cf.sign_flag = True
									#cv2.imwrite(f"/home/ubuntu/Downloads/THOR_237/sign_detection/ketqua/img{k}.jpg",roi)
								break					
					except:
						pass	
					cf.sign_show = image
