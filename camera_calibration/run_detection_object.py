import os
from camera_realworldxyz import camera_realtimeXYZ
import cv2 
from jetson_utils import videoSource, videoOutput, Log
import numpy as np
from time import sleep
from jetson_inference import detectNet
import json

robot_home = [31.3, 6.584+2.8] #home position of the robot wrt (0,0) (corner of the paper)
# [ 0, 300] , needs to be updated if scaled paper is moved 
# could add x-axis or y-axis offset between any of the objects and the soft gripper if offset is consistant 
cameraXYZ = camera_realtimeXYZ()

input_video = videoSource("/dev/video0")
sleep(1)

# load the object detection network
#net = detectNet('ssd-mobilenet-v2', .5)
net = detectNet(argv=['--model=/home/rsl/robot/custom2/ssd-mobilenet.onnx',
                '--labels=/home/rsl/robot/custom2/labels.txt',
                  '--output-bbox=boxes', '--input-blob=input_0', '--output-cvg=scores', '--clustering=0.4'])

id_to_label = {}
label_to_id = {}
with open('/home/rsl/robot/custom2/labels.txt', 'r') as f:
    lines = f.read().split('\n')
    for i, line in enumerate(lines):
        id_to_label[i] = line
        label_to_id[line] = i


img = input_video.Capture()
detections = net.Detect(img, overlay="box,labels,conf")
im = np.array(img)

print()
locations = {name: (id, (0,0), 0) for name, id in label_to_id.items()} 
for detection in detections:
    XYZ = cameraXYZ.calculate_XYZ(*detection.Center) #take list and convert to positional argument of a function
    print('Object ID:', detection.ClassID)
    print('Object Image Center:', detection.Center)
    print('Object Real-World Center:', XYZ[:2,0])
    print()
    x = int(detection.Center[0])
    y = int(detection.Center[1])
    # put circle cooridnates wrt global origin (base frame) on the image
    cv2.putText(im,"X,Y: "+str(cameraXYZ.truncate(XYZ[0],2))+","+str(cameraXYZ.truncate(XYZ[1],2)),(x,y+14),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2) #pixel to real world coordinates
    # put circle coordinates wrt to dexarm origin (robot frame) on the image
    cv2.putText(im,"X,Y: "+str(np.round(XYZ[0][0]-robot_home[0],0))+","+str(np.round(XYZ[1][0]-robot_home[1],0)),(x,y+28),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2) # pixel to dexarm coordinates
    # draw the circle on the image
    cv2.circle(im,(x,y),3,(0,255,0),2)

    obj_id = detection.ClassID
    name = id_to_label[detection.ClassID]
    pos_x = np.round(XYZ[0][0]-robot_home[0],0)
    pos_y = np.round(XYZ[1][0]-robot_home[1],0)
    locations[name] = (obj_id, (pos_x, pos_y), 1)
    
cv2.imwrite('detection.png',cv2.cvtColor(im, cv2.COLOR_BGR2RGB)) #save annotations + camera image to disk
with open('locations.json', 'w') as f:
  json.dump(locations, f)

