#!/usr/bin/python3
# license removed for brevity
import rospy
from geometry_msgs.msg import Point
import platform
#print(platform.python_version())
import sys
sys.path.append('/home/rsl/robot/camera_calibration')
from camera_realworldxyz import camera_realtimeXYZ
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, Log
import json
import argparse

global obj_id, pos_x, pos_y

obj_id = 0
pos_x = 0
pos_y = 0

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
input_video = videoSource("/dev/video0")
output_video = videoOutput(args.output, argv=sys.argv)

#with open('original.txt', 'r') as f:
#     labels = f.read().split('\n')
#     labels = [x if x == 'bottle' else 'void' for x in labels]
#with open('labels.txt', 'w') as f:
#     f.write('\n'.join(labels))

# load the object detection network
# net = detectNet(args.network, sys.argv, args.threshold)
net = detectNet(argv=['--model=/home/rsl/robot/custom2/ssd-mobilenet.onnx',
                '--labels=/home/rsl/robot/custom2/labels.txt',
                  '--output-bbox=boxes', '--input-blob=input_0', '--output-cvg=scores', '--clustering=0.4'])
id_to_label = {}
label_to_id = {}
with open('/home/rsl/robot/custom2/labels.txt', 'r') as f: #open .txt file
    lines = f.read().split('\n') #split text by newline characters, which results in a list (lines) where each element is a label name
    for i, line in enumerate(lines): #this loop iterates through the lines list, where i is the index (used as the class ID and line is the label name
        id_to_label[i] = line #associates each class ID (i) with its corresponding label name (line)
        label_to_id[line] = i #associates each label name (line) with its class ID (i)
#net = detectNet(model='ssd-mobilenet-v2', labels="labels.txt")

# note: to hard-code the paths to load a model, the following API can be used:
#
# net = detectNet(model="model/ssd-mobilenet.onnx", labels="model/labels.txt", 
#                 input_blob="input_0", output_cvg="scores", output_bbox="boxes", 
#                 threshold=args.threshold)
# remote:  (397.891, 339.785)


def main_prog():
    global obj_id, pos_x, pos_y

    pub = rospy.Publisher('object_def', Point, queue_size=5)
    rospy.init_node('vision_core', anonymous=True)
    rate = rospy.Rate(4) 
    while not rospy.is_shutdown():


        img = input_video.Capture()

        if img is None: # timeout
            continue  
        
    # detect objects in the image (with overlay)
        detections = net.Detect(img, overlay=args.overlay)
	#objects=[]
    # print the detections
        #print("detected {:d} objects in image".format(len(detections)))   
        locations = {name: (id, (0,0), 0) for name, id in label_to_id.items()} 
        for detection in detections:
            #print(f'{id_to_label[detection.ClassID + 1]}, {detection.Center}')
            #print(detection.ClassID)
            #pos_x = detection.Center[0]
            XYZ = cameraXYZ.calculate_XYZ(*detection.Center)
            pos_x = round(XYZ[0,0],2)
            pos_y = round(XYZ[1,0],2)
            obj_id = detection.ClassID
            name = id_to_label[detection.ClassID]
            locations[name] = (obj_id, (pos_x, pos_y), 1) #update locations dictionary for detected object's label with its ID, centroid, and detection flag 1.

            obj_def = Point()
            obj_def.x = pos_x
            obj_def.y = pos_y
            obj_def.z = obj_id + 1
            pub.publish(obj_def)
			#obj = String()
						
			#objects.append([pos_x,pos_y,obj_id+1])
		#obj.data = objects
            #rospy.loginfo(hello_str)
        with open('/home/rsl/sari_ws/src/voice_core/src/locations.json', 'w') as f:
             json.dump(locations, f)


    # render the image
        output_video.Render(img)

    # update the title bar
        output_video.SetStatus("{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS()))



        #obj_
        
        #def = Point()

        #obj_def.x = pos_x
        #obj_def.y = pos_y
        #obj_def.z = obj_id + 1
        
        #rospy.loginfo(hello_str)
        #pub.publish(obj_def)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main_prog()
    except rospy.ROSInterruptException:
        pass
