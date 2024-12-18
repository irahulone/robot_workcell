import cv2
import numpy as np
import time


class image_recognition:
    
    def __init__(self,print_status=True, write_images=False,
                 image_Path="/home/pi/Desktop/Captures/",testing_Path="/home/pi/Desktop/Captures/",
                 preview_images=False,preview_autoclose=True,print_img_labels=True):
        
        self.IMGDIR=image_Path
        self.TESTDIR=testing_Path
        self.PREVIEW_IMAGES=preview_images
        self.PREVIEW_AUTOCLOSE=preview_autoclose
        self.PRINT_STATUS=print_status
        self.PRINT_IMG_LABELS=print_img_labels
        self.WRITE_IMAGES=write_images
	
	#detection process fine-tuning

        #valid contour parameters limits (in pixels)
        self.MIN_AREA=900 #30x30
        self.MAX_AREA=90000 #300x300
            #aspect ratio width/height
        self.MIN_ASPECTRATIO=1/5
        self.MAX_ASPECTRATIO=5

        #OstuSensitivity
        self.OtsuSensitivity=22

    def test_objectDetect(self,bgFile,targetFile):

        img=cv2.imread(self.TESTDIR+bgFile+".jpg") #loads background image for comparison
        bg=cv2.imread(self.TESTDIR+targetFile+".jpg") #loads target image where objects will be detected against the background

        self.run_detection(img,bg,True)

    def run_detection(self,img,bg,testRun=False): #performs main object detection process
           
        obj_count, contours_detected, contours_validindex=self.detectObjects(img,bg) #detect objects

        obj_count, detected_points, img_output=self.detectionOutput(img,obj_count,contours_validindex,contours_detected) #generate output details
       
        return obj_count, detected_points, img_output #return results
    
    def detectObjects(self, image, bg_img,externalContours=True): #Isolates objects by comparing an image to its background and then identifies contours based on shape and location criteria

        img=image.copy()  # Create a copy of the input image to prevent modifications to the original image during processing.     
        background_img=bg_img.copy()


        # Process Image Difference
        diff=self.calculateDifference_Otsu(img,background_img) #calculate the difference between img and background_img using the Otsu thresholding method.

        # ///////////// Find the Contours
        # use RETR_EXTERNAL for only outermost contours... use RETR_TREE for all the full hierarchy (all contours including nested contours)
        if externalContours==True:
            contours_detected, hierarchy = cv2.findContours(diff, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        else:
            contours_detected, hierarchy = cv2.findContours(diff, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        #calculate key variables
        height, width, channels = img.shape

        # /////// identify the VALID Contours
        contours_validindex= self.identify_validcontours(contours_detected,height,width) #identify valid contours based on criteria such as area and aspect ratio
        obj_count=len(contours_validindex) #Count the number of valid contours
        self.printStatus("valid contours "+ str(obj_count)) #print the number of valid contours found, providing feedback on the number of objects detected in the image.

        return obj_count, contours_detected, contours_validindex

    def detectionOutput(self, image, obj_count, validcontours, diff_contours): # generates output based on detected contours, annotating the image and gathering key information

        img_output=image.copy() #store the visual output with rectangles and labels drawn around detected objects.

        detected_points=[] #Initializes an empty list to store information about each detected object, including position, dimensions, and centroid coordinates.


        if (len(validcontours)>0): #checks if any valid contours are detected
            for i in range(0,len(validcontours)): #Loop over each valid contour, where i is the index
                cnt=diff_contours[validcontours[i]] #Retrieves the specific contour from diff_contours based on the index stored in validcontours

                #get rectangle detected_points
                x,y,w,h=cv2.boundingRect(cnt)
                
                #get centroid
                M=cv2.moments(cnt) #Calculates the spatial moments of the contour used to compute centroid
                cx=int(M['m10']/M['m00']) #M['m10'] and M['m01'] are spatial moments, M['m00'] is the area of the contour
                cy=int(M['m01']/M['m00'])
                
                self.printStatus("point number "+str(i))  #print details about each detected point, such as its index, 
                self.printStatus(str(cx)+", "+str(y)) # centroid coordinates
                self.printStatus("x: "+str(x)+" y: "+str(y)+" w: "+str(w)+" h: "+str(h)) #and bounding box dimensions

                #draw retangle
                cv2.rectangle(img_output,(x,y),(x+w,y+h),(0,255,0),2) #Draw a green rectangle around the detected object on img_output using the bounding box coordinates (x, y) and (x + w, y + h).
                #draw center
                cv2.circle(img_output,(cx,cy),3,(0,255,0),2) #Draw a green circle at the centroid (cx, cy) of the detected object on img_output, indicating the center.

                if self.PRINT_IMG_LABELS ==True: #Check if image labels should be printed. If True, the next lines add text labels to the annotated image.
                    
                    #image,text,font,bottomleftconrner,fontscale,fontcolor,linetype 
                    #The first line labels the object number, while the second line labels the coordinates (cx, cy) of the centroid.
                    #Truncate is used to format the coordinates with a specified decimal precision.
                    cv2.putText(img_output,"Point "+str(i),(x-w,y+h),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
                    cv2.putText(img_output,"cx,cy: "+str(self.truncate(cx,2))+","+str(self.truncate(cy,2)),(x-w,y+h+9),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1)
                #Store bounding box coordinates and centroid data about each detected object in the points list 
                points=[x,y,w,h,cx,cy] 
                detected_points.append(points) #append points to detected_points for later reference.

        if (obj_count>1 or len(validcontours)==0):               
            self.previewImage("Multiple Objects Detected",img_output)
            one_object=False
        else:
            self.previewImage("One Objects Detected",img_output)
            one_object=True


        return obj_count, detected_points, img_output

    def truncate(self, n, decimals=0):
        n=float(n) #convert to float
        multiplier = 10 ** decimals #calculates multiplier
        return int(n * multiplier) / multiplier #truncate the number

    def writeImage(self,filename,image,testdir=False):
        if self.WRITE_IMAGES==True:  #If True, the function proceeds to save the image.
            if testdir==False: #determine save directory
                cv2.imwrite(self.TESTDIR+filename,image)
            else:
                cv2.imwrite(self.IMGDIR+filename,image)

                
    def readImage(self,imgfile):

        img=cv2.imread(imgfile)

        return img


                
    def printStatus(self,text):

        if self.PRINT_STATUS==True:
            print(text)
            
    
    def previewImage(self, text, img):
        if self.PREVIEW_IMAGES==True:
            #show full screen
            cv2.namedWindow(text, cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(text,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

            cv2.imshow(text,img)
            if self.PREVIEW_AUTOCLOSE==True:
                cv2.waitKey(2000)
                cv2.destroyAllWindows()
    
            else:
                cv2.waitKey(0)
                cv2.destroyAllWindows()

    def calculateDifference_method1(self,img,background_img): #computes the difference between an image and a background image using a sequence of grayscale conversion, difference calculation, blurring, and thresholding to isolate foreground objects. 
        
        # Object Recognition Thresholds (pixel intensities)
        diff_low_t=45
        diff_high_t=255

        self.previewImage("Original Image [Diff method1]",img) #If PREVIEW_IMAGES is enabled, displays the original image to provide a reference before any processing.

       # In this approach, The order is : Gray>Difference>Blur>Treshold>Blur.

        # Background - Gray 
        background_img_gray=cv2.cvtColor(background_img, cv2.COLOR_BGR2GRAY)
        self.previewImage("1 Background Gray",background_img_gray)

        # Image - Gray
        img_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.previewImage("2 Image Gray",img_gray)
         
        # Calculate Difference
        diff_gray=cv2.absdiff(background_img_gray,img_gray)
        self.previewImage("3 Pre-Diff",diff_gray)

        # Diff Blur (Blur the Difference Image)
        diff_gray_blur = cv2.GaussianBlur(diff_gray,(5,5),0) #5x5 pixels (larger kernel size = stronger blurring) and 0 is std dev in x, and y directions in order to let OpenCV automatically calculate an optimal value based on the kernel size
        self.previewImage("4 Pre-Diff Blur",diff_gray_blur)

        #========= Threshold :: this is a manual calibratin point in this approach (Threshold the Blurred Difference)
        ret,diff_tresh = cv2.threshold(diff_gray_blur,diff_low_t,diff_high_t,cv2.THRESH_BINARY)
        self.previewImage("5 Image Treshold",diff_tresh)

        #Treshold Blur (Blur the Thresholded Image)
        diff = cv2.GaussianBlur(diff_tresh,(5,5),0)
        self.previewImage("6 Image Treshold",diff)
        

        return diff

    def calculateDifference_method2(self,img,background_img): #uses grayscale conversion, blurring, thresholding, and differencing to isolate foreground objects
        
        # Object Recognition Tresholds
        bg_low_t=0
        bg_high_t=255
        img_low_t=120
        img_high_t=255

        self.previewImage("Original Image [Diff method2]",img)

        # In this approach, The order is: Gray>Blur>Treshold>Difference.


        # Background - Gray
        background_img_gray=cv2.cvtColor(background_img, cv2.COLOR_BGR2GRAY)
        self.previewImage("1 Background Gray",background_img_gray)

        # Background - Blur #Apply Gaussian Blur to Background
        background_img_blur = cv2.GaussianBlur(background_img_gray,(5,5),0)
        self.previewImage("2 Background Blur Gray",background_img_blur)       

        # Background - Treshold #Applies an inverted binary threshold to the blurred background image
        ret,background_img_tresh = cv2.threshold(background_img_blur,bg_low_t,bg_high_t,cv2.THRESH_BINARY_INV)
        self.previewImage("3 Background Treshold",background_img_tresh)

        # Image - Gray
        img_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.previewImage("4 Image Gray",img_gray)

        # Image - Blur #Apply Gaussian Blur to Target Image:
        img_blur = cv2.GaussianBlur(img_gray,(5,5),0)
        self.previewImage("5 Image Blur Gray",img_blur,testingPreviews)
        
        # Image - Treshold
        #========= Threshold :: this is a manual calibratin point in this approach
        ret,img_tresh = cv2.threshold(img_blur,img_low_t,img_high_t,cv2.THRESH_BINARY_INV)
        self.previewImage("6 Image Treshold",img_tresh)

        # Calculate Difference
        diff=cv2.absdiff(background_img_tresh,img_tresh)
        self.previewImage("7 Diff",diff,testingPreviews)    

        return diff


    def calculateDifference_Otsu(self,img,background_img):
        
        # Object Recognition Tresholds
        diff_low_t=45
        diff_high_t=255


        self.previewImage("Original Image [Diff Otsu]",img)

        # Background - Gray
        background_img_gray=cv2.cvtColor(background_img, cv2.COLOR_BGR2GRAY)
        self.previewImage("1 Background Gray",background_img_gray)

        # Image - Gray
        img_gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.previewImage("2 Image Gray",img_gray)

        # Calculate Difference
        diff_gray=cv2.absdiff(background_img_gray,img_gray)
        self.previewImage("3 Pre-Diff",diff_gray)

        # Diff Blur
        diff_gray_blur = cv2.GaussianBlur(diff_gray,(5,5),0)
        self.previewImage("4 Pre-Diff Blur",diff_gray_blur)

            #========= Otsu automatically finds the right threhosld, calibration not needed.

        # find otsu's threshold value with OpenCV function
        ret, otsu_tresh = cv2.threshold(diff_gray_blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU) #determine optimal threshold value ret and apply it to create binary thresholded image
        self.previewImage("5 Otsu Treshold",otsu_tresh)

        self.printStatus("Otsu defined treshold value is %d" % ret)

        if (ret < self.OtsuSensitivity):
            #discard image since if this this is the case, threshold is not strong enough to distinguish objects clearly.
            #make the difference zero by subtracting backgrounds
            diff=cv2.absdiff(background_img_gray,background_img_gray) #no objects detected
        else:       
            #Treshold Blur
            diff = cv2.GaussianBlur(otsu_tresh,(5,5),0)
            self.previewImage("6 Image Treshold",diff) #final image displayed containing isolated regions of foreground objects


        return diff #processed difference image


    def identify_validcontours(self,cnt,height,width): # filters contours based on area, aspect ratio, and edge proximity, helping eliminate noise and identify contours that match the expected characteristics of the objects of interest (e.g., size and shape).

        #helps discard noise on contour detection.
        #this is calibrated for Legos in this example

        contours_validindex=[] #empty list that will store indices of valid contours
        contour_index=-1 #increments with each contour to track current contour's index

        for i in cnt: #iterates through each contour in list of detected contours

            contour_index=contour_index+1 #increments for each contour to track its position
            ca=cv2.contourArea(i) #calculates area of current contour which is used for size based filtering

            # Calculate W/H Ratio
            x,y,w,h = cv2.boundingRect(i) #calculates bounding box of contour

            aspect_ratio = float(w)/h #helps with filtering out contours that do not match expected shape

            # Flag as edge_noise if the object is at a Corner by checking if bounding box touches image's edges

            #height, width, channels = img.shape
            edge_noise=False
            if x==0:
                edge_noise=True
            if y==0:
                edge_noise=True
            if (x+w)==width: #check if right edge of bounding box aligns exactly with right edge of image
                edge_noise=True
            if (y+h)==height: #check if bottom edge of bounding box aligns exactly with bottom edge of image
                edge_noise=True
                       
            # DISCARD noise with measure if area not within parameters
            if ca>self.MIN_AREA and ca<self.MAX_AREA:

                # DISCARD as noise on ratio
                if aspect_ratio>=self.MIN_ASPECTRATIO and aspect_ratio<=self.MAX_ASPECTRATIO:
  
                    # DISCARD if at the Edge
                    if edge_noise==False:
                         contours_validindex.append(contour_index)

        return contours_validindex

    def square_Crop(self,cnt,crop_img,contour_img,height,width): # modify contour’s bounding box to make it square, ensuring compatibility with models or processes that require square inputs (e.g., InceptionV3). 

        #You need squares to use InceptionV3 :)

        x,y,w,h = cv2.boundingRect(cnt) #calculate bounding box

        #print(x,y,w,h)

        # EXPAND THE CONTOUR by 15% in both dimensions to capture a margin around the contour.
        #Ensures y and x don’t go below zero, so the bounding box stays within image bounds.
        adjust=0.15 
        y=int(y-((h*adjust)/2)) #y and x are shifted outward by half of the expansion margin.
        if y<0:
            y=0
        x=int(x-((w*adjust)/2))
        if x<0:
            x=0
        w=int(w*(1+adjust)) #w and h are increased by 15%.
        h=int(h*(1+adjust))

        # CHECK TO SEE IF EXPANDED CONTOUR IS IN BOUNDS
        if y<0: y=0
        if x<0: x=0
        if (x+w)>width: w=width-x #Adjusts w and h to keep the expanded bounding box within the image dimensions 
        if (y+h)>height: h=height-y        

        # SQUARE THE CONTOUR (Adjusts x, y, w, and h to ensure the bounding box is square)
        if w>h: 
            #ensure contour is centered by adjusting y to center the square vertically, then set h = w.
            y=int(y-((w-h)/2))
            if y<0: y=0
            #make a square
            h=w
            if (y+h)>height: y=height-h
        if h>w: #adjust x to center the square horizontally, then set w = h.
            x=int(x-((h-w)/2))
            if x<0: x=0
            w=h
            if (x+w)>width: x=width-w


        #draw & crop the bounding box
        crop_img = crop_img[y:y+h, x:x+w] #Crop area defined by the square bounding box

        cv2.rectangle(contour_img,(x,y),(x+w,y+h),(0,255,0),2) #Draw a green rectangle on contour_img to indicate the bounding box around the contour.

        return crop_img, contour_img #Returns the cropped square image and the image with the drawn bounding box.

    def square_rotatedCrop(self,cnt,crop_img,contour_img,height,width): # rotates and crops a region defined by a contour to create a square, centered and optionally rotated bounding box around the object.

        #You need squares to use InceptionV3 :)

        #This rectangle will reflect the rotation of the image.
        rect = cv2.minAreaRect(cnt) #Find Minimum-Area Rotated Rectangle

        
        img=crop_img.copy()
        #rect[0]: Center of the rectangle (x, y).
        #rect[1]: Width and height of the rectangle.
        #rect[2]: Rotation angle of the rectangle.

        r_cx=rect[0][0] #center x
        r_cy=rect[0][1] #center y
        r_width=rect[1][0] #width of rectangle
        r_height=rect[1][1] #height of rectangle
    
        #EXPAND THE RECTANGLE ==> CHECK TO SEE IF NOT OUT OF BOUNDS
        adjust=0.15+0.05 #Expands the rectangle by a 20% margin
        while True:
            #assume True in each iteration
            fits_inbounds=True
            #reduce adjustment each iteration
            adjust=adjust-0.05 #Adjust by reducing adjust each iteration until the expanded rectangle (new_rect) fits within image bounds
            if adjust==0: break # or until no more expansion is possible.

            newW=int(r_width*(1+adjust)) #Calculates the expanded width 
            newH=int(r_height*(1+adjust)) #Calculates the expanded height
            new_rect=(rect[0],[newW,newH],rect[2]) #construct a new rotated rectangle 
            nbox=cv2.boxPoints(new_rect) #converts rotated rectangle to a set of corner points which are checked to ensure they fit within the image bounds

            for i in range(nbox.shape[0]): # iterate over each point in nbox to check if it fits within the image bounds.  
                #x or y smaller than zero
                for j in range (nbox[i].shape[0]):
                    if nbox[i][j]<0:
                        fits_inbounds=False
                #x greater than picture width
                if nbox[i][0]>width:
                    fits_inbounds=False
                #y greater than picture height
                    
                if nbox[i][1]>height:
                    fits_inbounds=False
            #If all points are within bounds
            if fits_inbounds==True:
                rect=new_rect
                break
            else: #If any point is out of bounds, the expansion attempt is discarded
                #dimensions revert to the original width and height.
                newW=int(r_width)
                newH=int(r_height)

        #Enable same Orientation when pieces are asymetrical (e.g. long side if image is horizontal)
        if r_height>r_width:
            #flip w, h
            rect=(rect[0],[r_height,r_width],rect[2]+90)

        #Make the rectangle a square by setting both dimensions to the larger one
        if newW>newH:
            rect=(rect[0],[newW,newW],rect[2])
        if newH>newW:
            rect=(rect[0],[newH,newH],rect[2])

        #Draw the Bounding Box       
        boxdraw = cv2.boxPoints(rect)
        boxdraw = np.int0(boxdraw) #convert each element of boxdraw to an integer type
        cv2.drawContours(contour_img,[boxdraw],0,(0,0,255),3)

        # rotate img based on the Rectangle’s Angle:
        angle = rect[2]
        rows, cols = img.shape[0], img.shape[1]
        M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, 1) #To align the rectangle with the image axes.
        img_rot = cv2.warpAffine(img, M, (cols, rows)) #rotates img by angle, resulting in the aligned image.

        # rotate bounding box and Ensure Coordinates Are Non-Negative
        box = cv2.boxPoints(rect)      
        pts = np.int0(cv2.transform(np.array([box]), M))[0] #creates rotated coordinates
        pts[pts < 0] = 0

        #re-establish width and height on rotated image
        # height: number of rows (or pixels vertically) in the image
        # width: number of columns (or pixels horizontally) in the image 
        # channel: number of color channels in the image.
        width, height, channel=img_rot.shape

        x=pts[1][1]
        xw=pts[0][1]
        w=xw-x
        y=pts[1][0]
        yh=pts[2][0]
        h=yh-y

        # CHECK TO SEE IF EXPANDED CONTOUR IS IN BOUNDS
        if y<0: y=0
        if x<0: x=0
        if (xw)>width: w=width-x
        if (yh)>height: h=height-y        

        # SQUARE THE CONTOUR
        if w>h:
            #make a square
            h=w
            if (y+h)>height: y=height-h
        if h>w:
            w=h
            if (x+w)>width: x=width-w

        # Crop the Image
        crop_img = img_rot[x:x+w,
                           y:y+h] #Crops the aligned, square region from img_rot using the coordinates calculated
        
        return crop_img,contour_img,r_width,r_height


#imgdir="/home/pi/Desktop/Captures/"

#(self,print_status=True, write_images=False,
#                 image_Path="/home/pi/Desktop/Captures/",testing_Path="/home/pi/Desktop/Captures/",
#                 preview_images=False,preview_autoclose=True):
        
#imageRec=image_recognition(True,True,imgdir,imgdir,True,True)
        
#imageRec.test_objectDetect("undst_bg","undst_cam")
