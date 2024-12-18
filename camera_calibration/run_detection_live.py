#!/usr/bin/env python3
#
# Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#

import sys
import argparse

sys.path.append('/home/rsl/robot/camera_calibration')
from time import time, sleep
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, Log, cudaDeviceSynchronize
from camera_realworldxyz import camera_realtimeXYZ

cameraXYZ = camera_realtimeXYZ()

# parse the command line
parser = argparse.ArgumentParser(description="Locate objects in a live camera stream using an object detection DNN.", 
                                 formatter_class=argparse.RawTextHelpFormatter, 
                                 epilog=detectNet.Usage() + videoSource.Usage() + videoOutput.Usage() + Log.Usage())

parser.add_argument("input", type=str, default="", nargs='?', help="URI of the input stream")
parser.add_argument("output", type=str, default="", nargs='?', help="URI of the output stream")
parser.add_argument("--network", type=str, default="ssd-mobilenet-v2", help="pre-trained model to load (see below for options)")
parser.add_argument("--overlay", type=str, default="box,labels,conf", help="detection overlay flags (e.g. --overlay=box,labels,conf)\nvalid combinations are:  'box', 'labels', 'conf', 'none'")
parser.add_argument("--threshold", type=float, default=0.5, help="minimum detection threshold to use") 

try:
	args = parser.parse_known_args()[0]
except:
	print("")
	parser.print_help()
	sys.exit(0)


# create video sources and outputs
#input_video = videoSource(args.input, argv=sys.argv)

output_video = videoOutput(args.output, argv=sys.argv)
input_video = videoSource("/dev/video0")

# load the object detection network
#net = detectNet(args.network, sys.argv, args.threshold)
net = detectNet(argv=['--model=/home/rsl/robot/custom2/ssd-mobilenet.onnx',
                '--labels=/home/rsl/robot/custom2/labels.txt',
                  '--output-bbox=boxes', '--input-blob=input_0', '--output-cvg=scores', '--clustering=0.4'])
#net = detectNet(model='ssd-mobilenet-v2', labels="labels.txt")

# note: to hard-code the paths to load a model, the following API can be used:
#
# net = detectNet(model="model/ssd-mobilenet.onnx", labels="model/labels.txt", 
#                 input_blob="input_0", output_cvg="scores", output_bbox="boxes", 
#                 threshold=args.threshold)
# remote:  (397.891, 339.785)
#from labels import id_to_label, label_to_id

id_to_label = {}
label_to_id = {}
with open('/home/rsl/robot/custom2/labels.txt', 'r') as f:
    lines = f.read().split('\n')
    for i, line in enumerate(lines):
        id_to_label[i] = line
        label_to_id[line] = i

#print(id_to_label)
#print(label_to_id)
#quit()
start = time()
# process frames until EOS or the user exits
while True:
    # capture the next image
    #sleep(0.015) #force to elapse 15 ms more
    img = input_video.Capture()
    print('Elapsed', (time()- start)*1000) #print measured time elapsed between captures
    start=time() #update start to the current time, allowing the next iteration to measure time accurately.

    if img is None: # timeout
        continue  
        
    # detect objects in the image (with overlay)
    detections = net.Detect(img, overlay=args.overlay)

    # print the detections
    print("detected {:d} objects in image".format(len(detections)))    
    for detection in detections:                
        print(f'{id_to_label[detection.ClassID]}, {detection.Center}')
        XYZ = cameraXYZ.calculate_XYZ(*detection.Center) #comment this line out to make program run more like detectnet        
        print('Object Real-World Center:', XYZ[:2,0]) #comment this line out to make program run more like detectnet ,:0 --> x , :1 -->y, :2 --> z
        print()
        #x_b=1080-detection.Center[0]
        #x_b=x_b*0.481
        #print(x_b)
        #print(f'{detection.Center[0]}')
    
    # render the image
    output_video.Render(img)

    # update the title bar
    output_video.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()))

    # print out performance info
    # net.PrintProfilerTimes()

    # exit on input/output EOS
    if not input_video.IsStreaming() or not output_video.IsStreaming():
        break
