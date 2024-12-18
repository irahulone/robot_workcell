# Speech Enabled Perception and Action of a 4 Degree of Freedom (4DoF) DexArm Robot


## Overview

Learn about this robot here and perhaps buy a kit:  https://rotrics.com/

Overview of the Robot: https://rotrics.com/pages/dexarm-overview


## Initial Camera Calibration

**initial_camera_calibration.py** follows very closely the OpenCV documentation and saves the output matrixes.

**camera_data** is where the camera calibration files are stored.

**get_images.py** Calibrates the camera using a chessboard to understand how the camera distorts and translates real-world coordinates into 2D image coordinates, enabling corrections for lens distortion and improving spatial accuracy.

**initial_perspective_calibration.py** enables you to calibrate the camera to the perspective AND enables you to check the accuracy of your calibration.

## Image Detection and XYZ calculation

**image_recognition_singlecam.py** enables you to do background extraction and object detection.

**camera_realdworldxyz.py** takes the u,v pixel points of the object detected and translates it to real world coordinates (for the arm to grab).

**run_detection_circles.py** detects the circular pattern on a sheet of paper, annotates both pixel and real-world coordinates relative to the base frame and saves results to an image called "im_circle.png"

**run_detection_object.py** enables you to do  object detection, capturing a single image containing current objects present in workspace which is outputted as "detection.png". This image also contains information such as the object's centroid coordinates relative to the base frame as well as their calculated distance from the dexarm's home position.

**run_detection_live.py** enables you to continuously stream object detection in real time, print pixel and real world centroid coordinates, as well as calculate the time delay between centroid readings.


## To Run the Program: 

1. Get 40 images using chessboard
```
/usr/bin/python3 get_images.py
```
2. Tape the page containing the circular pattern to the upper-left side of the real world origin

3. Run circle detection 
```
/usr/bin/python3 run_detection_circles.py
```
4. Fill the values inside `initial_perspective_calibration.py` with pixel coordinates reported by `im_circle.png`

5. Run the following programs in order
```
/usr/bin/python3 initial_camera_calibration.py
/usr/bin/python3 initial_perspective_calibration.py
```
Check error values to be reasonable

6. Rerun circle detection and check if it makes sense
```
/usr/bin/python3 run_detection_circles.py
```
7. Run object detection:
```
/usr/bin/python3 run_detection_object.py

