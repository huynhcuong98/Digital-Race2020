import json
import os
import glob
import csv
import cv2
import pandas as pd
import matplotlib.pyplot as plt
data_path = pd.read_csv('./data5/data.csv')
path = './image/'
data=[]
auto = True
i = 1
while True:
	try:
		image_path = os.path.join('./data5/'+'frame_'+str(data_path.iloc[i, 0])+'.png')
		img = cv2.imread(image_path)
		a=data_path.iloc[i, 1]
		print(image_path)
		print(a)
		a=int(a)
		cv2.circle(img,(a,165),3,(255,255,0),3)
		cv2.imshow("frame", img)
		if auto:
			key = cv2.waitKey(5)
			if key & 0xFF == ord('q'):
				break
			if key & 0xFF == ord(' '):
				auto = not auto
			else:
				i+= 1
		else:
			key = cv2.waitKey(0)
			if key & 0xFF == ord('d'):
				i += 1
			if key & 0xFF == ord('a'):
				i -= 1
			if key & 0xFF == ord('q'):
				break
			if key & 0xFF == ord(' '):
				auto = not auto
	except:
		pass
	#data.append(a)
plt.hist(data)
plt.show()
