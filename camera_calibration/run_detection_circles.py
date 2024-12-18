import os
os.chdir('/home/rsl/robot/camera_calibration')
from camera_realworldxyz import camera_realtimeXYZ
import cv2
from jetson_utils import videoSource, videoOutput, Log
import numpy as np
from time import sleep

cameraXYZ = camera_realtimeXYZ() #initialize to convert 2D pixel to 3D world coordinates

input_video = videoSource("/dev/video0")

#img = input_video.Capture()
#background = np.array(img)  
#cv2.imwrite(f'background.png', background)  
background = cv2.imread('background.png')

img = input_video.Capture()
sleep(1)
im = np.array(img)    
cv2.imwrite(f'im.png', im)

#not real world (pixels)
cameraXYZ.load_background(background)
im_circle, objs = cameraXYZ.detect_xyz(im, False)
cv2.imwrite(f'im_circle.png', im_circle)

#real world XYZ
im_circle, objs = cameraXYZ.detect_xyz(im, True)
cv2.imwrite(f'im_circle.png', im_circle)

