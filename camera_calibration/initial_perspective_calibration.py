#https://docs.opencv.org/3.3.0/dc/dbb/tutorial_py_calibration.html
# The process for homogenous transformation is to take care of the camera distortion and conversion of
# pixels to real world coordinates 
# this happens through two processes: camera calibration and perspective calibration
# in camera calibration, the distortion introduced by the camera is found.
# in perspective calibration, a set of matrices for scaling, translation and rotation to convert pixels to real world cooridnates is found. 
# camera calibration is done using chessboard images
# perspective calibration is done using 6 circles where the pixel coordinates are determined using object detection and
# real-word cooridnates using manual measurement and then the corresponding translation, rotation, and scaling matrices are calculated and used in calculate_XYZ function of 
# camera_realtimeXYZ class

import numpy as np
import cv2
import glob
import camera_realworldxyz
import os

calculatefromCam=True

imgdir="/home/rsl/robot/camera_calibration"
os.chdir(imgdir)

writeValues=True

#test camera calibration against all points, calculating XYZ

#load camera calibration data when doing chessboard 
savedir="camera_data/"
cam_mtx=np.load(savedir+'cam_mtx.npy') #intrinsic camera matrix
dist=np.load(savedir+'dist.npy') #distortion coefficients for lens correction
newcam_mtx=np.load(savedir+'newcam_mtx.npy') #adjusted intrinsic matrix post correction
roi=np.load(savedir+'roi.npy') #region of interest after undistortion


#load center points from New Camera matrix
cx=newcam_mtx[0,2]
cy=newcam_mtx[1,2]
fx=newcam_mtx[0,0]
print("cx: "+str(cx)+",cy "+str(cy)+",fx "+str(fx))

#Define real-world 3D coordinates corresponding to calibration points
#MANUALLY INPUT YOUR MEASURED POINTS HERE
#ENTER (X,Y,d*)
#d* is the distance from a point to the camera lens. (d* = Z for the camera center)
#Z is calculated in the next steps after extracting the new_cam matrix




total_points_used=10 #world center + 9 world points
h = 28.575 #Z-value (distance from camera to the calibration plane)
X_center=10.9 #X-coordinate of the world center
Y_center=10.7 #Y-coordinate of the world center
Z_center=h #Z-value
worldPoints=np.array([[X_center,Y_center,Z_center], #global frame (arbitary) which is chosen to be the corner of the paper grid
                       [8.75,5.9,h],
                       [19.1,5.9,h],
                       [29.6,5.9,h],
                       [8.75,13.962,h],
                       [19.1,13.962,h],
                       [29.6,13.962,h],
                       [8.75,22,h],
                       [19.1,22,h],
                       [29.6,22,h]], dtype=np.float32)

#MANUALLY INPUT THE DETECTED IMAGE COORDINATES HERE

#[u,v] center + 9 Image points
imagePoints=np.array([[cx,cy], # image frame 
                       [200,100],
                       [531,104],
                       [859,106],
                       [191,355],
                       [527,358],
                       [858,362],
                       [181,617],
                       [521,620],
                       [860,625]], dtype=np.float32)


#FOR REAL WORLD POINTS, CALCULATE Z from d*
# this assumes that the center of the paper is at the center of camera
# for i in range(1,total_points_used):
#     #start from 1, given for center Z=d*
#     #to center of camera
#     wX=worldPoints[i,0]-X_center
#     wY=worldPoints[i,1]-Y_center
#     wd=worldPoints[i,2]

#     d1=np.sqrt(np.square(wX)+np.square(wY))
#     wZ=np.sqrt(np.square(wd)-np.square(d1))
#     worldPoints[i,2]=wZ

print(worldPoints)

#print loaded calibration data for verification
#print(ret)
print("Camera Matrix")
print(cam_mtx)
print("Distortion Coeff")
print(dist)

print("Region of Interest")
print(roi)
print("New Camera Matrix")
print(newcam_mtx)
inverse_newcam_mtx = np.linalg.inv(newcam_mtx)
print("Inverse New Camera Matrix")
print(inverse_newcam_mtx)
if writeValues==True: 
    np.save(savedir+'inverse_newcam_mtx.npy', inverse_newcam_mtx)

print(">==> Calibration Loaded")

#Estimating the Camera Pose (Solving the Perspective-n-Point) problem
#ret is a boolean value (T/F) which indicates whether the function successfully found a solution for the camera pose
print("solvePNP")
ret, rvec1, tvec1=cv2.solvePnP(worldPoints,imagePoints,newcam_mtx,dist)

print("pnp rvec1 - Rotation")
print(rvec1)
if writeValues==True: 
    np.save(savedir+'rvec1.npy', rvec1)

print("pnp tvec1 - Translation")
print(tvec1)
if writeValues==True: 
    np.save(savedir+'tvec1.npy', tvec1)

print("R - rodrigues vecs")
R_mtx, jac=cv2.Rodrigues(rvec1) #Converts rotation vector to rotation matrix
print(R_mtx)
if writeValues==True: 
    np.save(savedir+'R_mtx.npy', R_mtx)

print("R|t - Extrinsic Matrix")
Rt=np.column_stack((R_mtx,tvec1)) 
print(Rt)
if writeValues==True: 
    np.save(savedir+'Rt.npy', Rt)

print("newCamMtx*R|t - Projection Matrix")
P_mtx=newcam_mtx.dot(Rt)
print(P_mtx)
if writeValues==True: 
    np.save(savedir+'P_mtx.npy', P_mtx)

#[XYZ1]



# This block calculates the accuracy of the scaling factor
# for each of those circles, I estimated the scaling factor by converting
# the real world points to pixel points and divided the estimated pixel points by the
# actual pixel points to get the scaling factor. I wanted the scaling factor to be the same 
# for all 9 points or it should have a small variance (error)

s_arr=np.array([0], dtype=np.float32)
s_describe=np.array([0,0,0,0,0,0,0,0,0,0],dtype=np.float32) #storage array for scaling factors calculated for each of the 10 calibration points

for i in range(0,total_points_used):
    print("=======POINT # " + str(i) +" =========================")
    
    print("Forward: From World Points, Find Image Pixel")
    XYZ1=np.array([[worldPoints[i,0],worldPoints[i,1],worldPoints[i,2],1]], dtype=np.float32) # i is index from 0 to 8 ,1 converts 3D world points into homogenous coordinates
    XYZ1=XYZ1.T
    print("{{-- XYZ1")
    print(XYZ1)
    suv1=P_mtx.dot(XYZ1)
    print("//-- suv1")
    print(suv1)
    s=suv1[2,0]    
    uv1=suv1/s
    print(">==> uv1 - Image Points")
    print(uv1)
    print(">==> s - Scaling Factor")
    print(s)
    s_arr=np.array([s/total_points_used+s_arr[0]], dtype=np.float32)
    s_describe[i]=s
    if writeValues==True: np.save(savedir+'s_arr.npy', s_arr)

    print("Solve: From Image Pixels, find World Points")

    uv_1=np.array([[imagePoints[i,0],imagePoints[i,1],1]], dtype=np.float32) #create homogenous representation of 2D image points
    uv_1=uv_1.T
    print(">==> uv1")
    print(uv_1)
    suv_1=s*uv_1
    print("//-- suv1")
    print(suv_1)

    print("get camera coordinates, multiply by inverse Camera Matrix, subtract tvec1")
    xyz_c=inverse_newcam_mtx.dot(suv_1) #map scaled image coordinates into camera's normalized 3D coordinate space using the inverse intrinsic matrix
    xyz_c=xyz_c-tvec1 #adjust coordinates for the camera's translation relative to the world
    print("      xyz_c")
    inverse_R_mtx = np.linalg.inv(R_mtx)
    XYZ=inverse_R_mtx.dot(xyz_c) #transform rotation matrix inverse from camera coordinate system to world coordinate system
    print("{{-- XYZ")
    print(XYZ)

    if calculatefromCam==True:
        cameraXYZ = camera_realworldxyz.camera_realtimeXYZ()
        cXYZ=cameraXYZ.calculate_XYZ(imagePoints[i,0],imagePoints[i,1])
        print("camXYZ")
        print(cXYZ)


s_mean, s_std = np.mean(s_describe), np.std(s_describe)

print(">>>>>>>>>>>>>>>>>>>>> S RESULTS")
print("Mean: "+ str(s_mean))
#print("Average: " + str(s_arr[0]))
print("Std: " + str(s_std))

print(">>>>>> S Error by Point")

for i in range(0,total_points_used):
    print("Point "+str(i))
    print("S: " +str(s_describe[i])+" Mean: " +str(s_mean) + " Error: " + str(s_describe[i]-s_mean))

