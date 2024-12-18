## Camera Calibration
Before running the vision subsystem, The camera should be calibrated such that the location of robot, camera and the grid paper are fixed with the base of the robot parallel to the grid paper. To calibrate the camera, Instructions provided in `camera_calibration/README.md` must be followed. 


## To run vision: 

- 1st terminal 

```roscore```

- 2st terminal

```
ls 
cd sari_ws
source devel/setup.bash
rosrun vision_core vision_node.py
```


- 3rd terminal 

```
rostopic echo /object_def
```

## To run voice: 

```
cd /home/rsl/robot_workcell/sari_ws/src/voice_core/src/
python voice_backend.py
```
Then copy paste generated ngrok url into alexa Frontend and deploy the code

## Robot Calibration: 
Additional instructions to set up the robot subsystem are available in section 2.3.3.2 of the project report.
