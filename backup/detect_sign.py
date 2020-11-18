import cv2
import numpy as np
import matplotlib.pyplot as plt
import joblib
import config as cf

#filename = 'HOGDescriptor.sav'
#clf = joblib.load(filename)



def extract_features(img):
    winSize = (20,20)
    blockSize = (20,20)
    blockStride = (1,1)
    cellSize = (5,5)
    nbins = 9
    hog = cv2.HOGDescriptor(winSize,blockSize,blockStride,cellSize,nbins)
    return hog.compute(img)
def nothing(x):
    pass
def sign():
	global clf
	kernel = np.ones((9,9),np.uint8)
	while( cf.running):
		image = cf.depth_show
		img1= image.copy()
		img2=image.copy()
		if cf.syn1:
			cf.syn1=False
			img1=cf.depth_show
			img1 = cv2.blur(img1,(5,5))
		    # image = cv2.imread("blue.jpg")
			hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
		    ##RED
		    # lower_red = np.array([170, 70, 50])
		    # upper_red = np.array([179, 255, 200])
		    #BLUE
			lower = np.array([108, 62, 42])
			upper = np.array([125, 250, 230])
		    # Morphological Transformations,Opening and Closing
			thresh = cv2.inRange(hsv,lower, upper)
		    #print(thresh.shape)
		    # combined=cv2.medianBlur(thresh,21)
			cf.depth=thresh
			cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
			try: 
				for c in cnts:
					(x, y, w, h) = cv2.boundingRect(c)
					(circle_pos, radius) = cv2.minEnclosingCircle(c)
					if radius>10 and radius <50:
			    # if radius >1:
						cv2.rectangle(cf.depth_show, (x, y), (x + w, y + h), (255, 255, 0), 2)
						img= image[y:y+h,x:x+w]
						cf.bounding=[x+8,y-10,w,h]
						cf.flag_sign=True  
		   	   # print(cf.bounding)        
			except:
				pass
'''
		    img2 = cv2.blur(img2,(5,5))
		    # image = cv2.imread("blue.jpg")
		    hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
		    ##RED
		    lower_red = np.array([170, 70, 50])
		    upper_red = np.array([179, 255, 200])
		    #BLUE
		    # lower_red = np.array([108, 62, 42])
		    # upper_red = np.array([125, 250, 230])
		    # Morphological Transformations,Opening and Closing
		    thresh_red = cv2.inRange(hsv,lower_red, upper_red)
		    thresh_red = cv2.dilate(thresh_red,kernel,iterations = 1)
		    #combined_red=cv2.medianBlur(thresh_red,3)
		    cnts = cv2.findContours(thresh_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		    #cv2.imshow('sssss',thresh_red)
		    cf.depth1= thresh_red
		    #cv2.waitKey(2)
		    try: 
			for c in cnts:
			    (x, y, w, h) = cv2.boundingRect(c)
			    (circle_pos, radius) = cv2.minEnclosingCircle(c)
			    if radius>15 and abs(w-h)<12 and radius<40:
			    # if radius >10:
				# print(abs(w-h))
				cv2.rectangle(cf.depth_show, (x, y), (x + w, y + h), (0, 0, 255), 2)
				img= image[y:y+h,x:x+w]                
				img = cv2.resize(img, (20,20)) 
				# img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
				#fd = extract_features(img)
				#fd=fd.reshape(1,-1)
				#a = clf.predict(fd)
				#print(a)
				#cv2.putText(image, str(a), (50,50), cv2.FONT_HERSHEY_SIMPLEX,  
				      # 0.5, (0, 0, 255), 2, cv2.LINE_AA)
		    except:
			pass
		    '''
		
