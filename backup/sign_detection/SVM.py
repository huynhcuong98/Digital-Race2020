
import cv2
import joblib
from skimage.feature import hog
############################



model_recblue = 'sign_detection/models/ver1.sav'
clf_recblue = joblib.load(model_recblue)

model_cirred = 'sign_detection/models/ver1.sav'
clf_cirred = joblib.load(model_cirred)

def predict_recblue(img):

    img = cv2.resize(img, (20,20))
    fd, hog_image = hog(img, orientations = 9, pixels_per_cell= (5,5), cells_per_block = (4,4), visualize = True, multichannel = True, block_norm='L2-Hys')
    fd=fd.reshape(1,-1)
    pre = clf_recblue.predict(fd)
    return pre[0]

def predict_cirred(img):
 
    img = cv2.resize(img, (20,20))
    fd, hog_image = hog(img, orientations = 9, pixels_per_cell= (5,5), cells_per_block = (4,4), visualize = True, multichannel = True, block_norm='L2-Hys')
    fd=fd.reshape(1,-1)
    pre = clf_cirred.predict(fd)
    return pre[0]

