import cv2
from jetson_utils import videoSource, videoOutput, Log
import numpy as np
import os
import shutil
from time import sleep
path = '/home/rsl/robot/camera_calibration/calibrations'
shutil.rmtree(path, ignore_errors=True) # remove files from the folder if any exist
os.makedirs(path, exist_ok=True) # create folder

input_video = videoSource("/dev/video0")

for i in range(40):
    # capture the next image
    img = input_video.Capture()
    sleep(1)
    im = np.array(img)    
    cv2.imwrite(f'{path}/example_{i:02d}.png', im)
    input('Please enter to continue')
    
