#!/usr/bin/env python3
import os
import sys
import copy
import re
import importlib
import numpy as np
import roslib
import rospy
import rospkg
import time
import sensor_msgs.msg
import nxp_gazebo.msg
from cv_bridge import CvBridge
import cv2
if cv2.__version__ < "4.0.0":
    raise ImportError("Requires opencv >= 4.0, "
                      "but found {:s}".format(cv2.__version__))
class TrackVision(object):

    def __init__(self):

        #setup CvBridge
        self.bridge = CvBridge()

        #Number of times to pyramid down the image before processing
        self.pyrDown = int(rospy.get_param("~pyr_down", 1))
        
        #Rectangualr area to remove from image calculation to 
        # eliminate the vehicle. Used as ratio of overall image width and height
        # "width ratio,height ratio"
        self.maskRectRatioWidthHeight = np.fromstring(
            rospy.get_param("~rwh", "0.4,0.45"), sep=',').astype(float)
        
        #Bool for generating and publishing the debug image
        self.debug = rospy.get_param("~vision_debug", False)

        #Pixy image size parameters
        self.pixyImageWidth = 72
        self.pixyImageHeight = 52
        
        #Time node started in UTC
        self.timeStart = time.time()
        
        #Subscribers
        self.imageSub = rospy.Subscriber(
            "/nxp_cupcar/Pixy2CMUcam5/image_raw", sensor_msgs.msg.Image, self.callback)
        
        #Publishers
        self.debugDetectionImagePub = rospy.Publisher(
            "DebugDetectionImage", sensor_msgs.msg.Image, queue_size=0)
        self.PixyVectorPub = rospy.Publisher(
            "PixyVector",nxp_gazebo.msg.PixyVector,queue_size=0)
        
    def findLines(self, passedImage):
        
        #convert image to grayscale
        passedImageGray = cv2.cvtColor(passedImage,cv2.COLOR_BGR2GRAY)
        
        #Image dimensions
        imageHeight, imageWidth = passedImageGray.shape[:2]
        
        #Threshold image black and white
        passedImageGrayThresh = cv2.bitwise_not(cv2.threshold(
            passedImageGray, 10, 255, cv2.THRESH_BINARY)[1])
        
        #Create image mask background
        maskWhite = np.ones(passedImageGrayThresh.shape[:2], dtype="uint8") * 255
        
        #calculate points to be masked based on provided ratio
        maskVehicleBoxTopLeftXY = (int(imageWidth*(1.0-self.maskRectRatioWidthHeight[0])/2.0), 
            int(imageHeight*(1.0-self.maskRectRatioWidthHeight[1])))
        
        #calculate points to be masked based on provided ratio
        maskVehicleBoxBottomRightXY = (int(imageWidth*(1.0+self.maskRectRatioWidthHeight[0])/2.0), 
            int(imageHeight))
        
        maskVehicle = cv2.rectangle(maskWhite,maskVehicleBoxTopLeftXY,
            maskVehicleBoxBottomRightXY,color=0,thickness=-1)
        
        #Mask out the area of the vehicle
        passedImageGrayThreshMasked = cv2.bitwise_and(passedImageGrayThresh, 
            passedImageGrayThresh, mask=maskVehicle)
        
        #Find contours
        cnts, hierarchy = cv2.findContours(passedImageGrayThreshMasked.copy(),
            cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        
        returnedImageDebug=passedImage
        

        if self.debug:

            #Border for vehicle mask
            returnedImageDebug = cv2.rectangle(returnedImageDebug,maskVehicleBoxTopLeftXY,
                maskVehicleBoxBottomRightXY, color=(128,128,0),thickness=3)

            #Calcualte background for pixy image space
            debugPixyMessageTopLeftXY = (int((imageWidth/2)-(self.pixyImageWidth/2)),
                int((imageHeight/2)-(self.pixyImageHeight/2)))

            #Create background for pixy image space
            returnedImageDebug = cv2.rectangle(returnedImageDebug,debugPixyMessageTopLeftXY,
                ((debugPixyMessageTopLeftXY[0]+self.pixyImageWidth),
                    (debugPixyMessageTopLeftXY[1]+self.pixyImageHeight)),(0,255,0),-1)

        #Max number of found contours to process based on area of return, largest returned first
        maxCnt = 2
        if len (cnts) < maxCnt:
            maxCnt = len(cnts)
        cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0:maxCnt]

        #initialize and/or clear existing found line vector array
        pixyScaledVectorArray = np.empty([0,4], int)

        for cn in cnt:
            #Find lines from contours using least square method
            [vectorX,vectorY,linePointX,linePointY] = cv2.fitLine(cn,cv2.DIST_L2,0,0.01,0.01)
            
            #Calculate line points to see if they exceeds any bounds of the image, correct if they do
            topLeftX = int((-linePointY*vectorX/vectorY)+linePointX)
            bottomRightX = int(((imageHeight-linePointY)*vectorX/vectorY)+linePointX)
            topLeftY = 0
            bottomRightY = imageHeight
            
            if (topLeftX < 0) and (bottomRightX > imageWidth):
                topLeftY = int(((-linePointX)*vectorY/vectorX)+linePointY)
                bottomRightY = int(((imageWidth-linePointX)*vectorY/vectorX)+linePointY)
                topLeftX = 0
                bottomRightX = imageWidth

            elif (topLeftX > imageWidth) and (bottomRightX < 0):
                topLeftY = int(((imageWidth-linePointX)*vectorY/vectorX)+linePointY)
                bottomRightY = int(((-linePointX)*vectorY/vectorX)+linePointY)
                topLeftX = imageWidth
                bottomRightX = 0

            elif (topLeftX < 0) and (bottomRightX < imageWidth):
                topLeftY = int(((-linePointX)*vectorY/vectorX)+linePointY)
                bottomRightY = imageHeight
                topLeftX = 0
                bottomRightX = int(((imageHeight-linePointY)*vectorX/vectorY)+linePointX)

            elif (topLeftX > 0) and (bottomRightX > imageWidth):
                topLeftY = 0
                bottomRightY = int(((imageWidth-linePointX)*vectorY/vectorX)+linePointY)
                topLeftX = int((-linePointY*vectorX/vectorY)+linePointX)
                bottomRightX = imageWidth

            elif (topLeftX > imageWidth) and (bottomRightX > 0):
                topLeftY = imageHeight
                bottomRightY = int(((imageWidth-linePointX)*vectorY/vectorX)+linePointY)
                topLeftX = int(((imageHeight-linePointY)*vectorX/vectorY)+linePointX)
                bottomRightX = imageWidth

            elif (topLeftX < imageWidth) and (bottomRightX < 0):
                topLeftY = int((-linePointX*vectorY/vectorX)+linePointY)
                bottomRightY = 0
                topLeftX = 0
                bottomRightX = int((linePointY)*(-vectorX/vectorY)+linePointX)

            #Scale into Pixy camera units
            topLeftXScaled = int(topLeftX*(self.pixyImageWidth/imageWidth))
            bottomRightXScaled = int(bottomRightX*(self.pixyImageWidth/imageWidth))
            topLeftYScaled = int(topLeftY*(self.pixyImageWidth/imageWidth))
            bottomRightYScaled = int(bottomRightY*(self.pixyImageWidth/imageWidth))

            #Append found line points to pixy found line vector array
            pixyScaledVectorArray = np.append(pixyScaledVectorArray,
                np.array([[topLeftXScaled,topLeftYScaled,bottomRightXScaled,bottomRightYScaled]]),axis=0)
            
            if self.debug:

                #Paint all the areas found in the contour
                cv2.fillPoly(returnedImageDebug,pts=[cn],color=(0,0,255))
                
                #Draw the found line in image space
                returnedImageDebug = cv2.line(returnedImageDebug,(topLeftX,topLeftY),
                    (bottomRightX,bottomRightY),(255,128,128),2)
                
                #Draw the found line in image space
                returnedImageDebug = cv2.line(returnedImageDebug,
                    (debugPixyMessageTopLeftXY[0]+topLeftXScaled,
                    debugPixyMessageTopLeftXY[1]+topLeftYScaled),
                    (debugPixyMessageTopLeftXY[0]+bottomRightXScaled,
                    debugPixyMessageTopLeftXY[1]+bottomRightYScaled),
                    (255,128,128),1)
                
                #Draw box point for top left XY for Pixy space debug image
                returnedImageDebug = cv2.rectangle(returnedImageDebug,
                    (debugPixyMessageTopLeftXY[0]+topLeftXScaled-1, 
                    debugPixyMessageTopLeftXY[1]+topLeftYScaled-1),
                    (debugPixyMessageTopLeftXY[0]+topLeftXScaled+1, 
                    debugPixyMessageTopLeftXY[1]+topLeftYScaled+1),
                    (0,0,255),-1)
                
                #Draw box point for bottom right XY for Pixy space debug image
                returnedImageDebug = cv2.rectangle(returnedImageDebug,
                    (debugPixyMessageTopLeftXY[0]+bottomRightXScaled-1,
                    debugPixyMessageTopLeftXY[1]+bottomRightYScaled-1),
                    (debugPixyMessageTopLeftXY[0]+bottomRightXScaled+1, 
                    debugPixyMessageTopLeftXY[1]+bottomRightYScaled+1),
                    (0,0,255),-1)
                
                #Write text for top left XY for Pixy space debug image
                returnedImageDebug = cv2.putText(
                    returnedImageDebug, '{:d},{:d}'.format(topLeftXScaled,topLeftYScaled), 
                    (debugPixyMessageTopLeftXY[0]+topLeftXScaled, 
                    debugPixyMessageTopLeftXY[1]+topLeftYScaled), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255,0,0), 1, cv2.LINE_AA)
                
                #Write text for bottom right XY for Pixy space debug image
                returnedImageDebug = cv2.putText(returnedImageDebug, '{:d},{:d}'.format(
                    bottomRightXScaled,bottomRightYScaled), 
                    (debugPixyMessageTopLeftXY[0]+bottomRightXScaled, 
                    debugPixyMessageTopLeftXY[1]+bottomRightYScaled), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255,0,0), 1, cv2.LINE_AA)

        
        #Pixy message for publication
        if len(pixyScaledVectorArray) > 0:
            timeStampMicroSeconds = (time.time()-self.timeStart)*1000000
            PixyVector_msg = nxp_gazebo.msg.PixyVector()
            PixyVector_msg.timestamp = int(timeStampMicroSeconds)
            PixyVector_msg.m0_x0 = pixyScaledVectorArray[0][0]
            PixyVector_msg.m0_y0 = pixyScaledVectorArray[0][1]
            PixyVector_msg.m0_x1 = pixyScaledVectorArray[0][2]
            PixyVector_msg.m0_y1 = pixyScaledVectorArray[0][3]
            PixyVector_msg.m1_x0 = 0
            PixyVector_msg.m1_y0 = 0
            PixyVector_msg.m1_x1 = 0
            PixyVector_msg.m1_y1 = 0
            if len(pixyScaledVectorArray) > 1:
                PixyVector_msg.m1_x0 = pixyScaledVectorArray[1][0]
                PixyVector_msg.m1_y0 = pixyScaledVectorArray[1][1]
                PixyVector_msg.m1_x1 = pixyScaledVectorArray[1][2]
                PixyVector_msg.m1_y1 = pixyScaledVectorArray[1][3]
            self.PixyVectorPub.publish(PixyVector_msg)

        return(returnedImageDebug)
      
    
    def callback(self, data):
        
        # Scene from subscription callback
        scene = self.bridge.imgmsg_to_cv2(data, "bgr8")
        
        #deep copy and pyramid down image to reduce resolution
        scenePyr = copy.deepcopy(scene)
        if self.pyrDown > 0:
            for i in range(self.pyrDown):
                scenePyr = cv2.pyrDown(scenePyr)
        sceneDetect = copy.deepcopy(scenePyr)

        #find lines function
        sceneDetected = self.findLines(sceneDetect)
        
        if self.debug:
            #publish debug image
            msg = self.bridge.cv2_to_imgmsg(sceneDetected, "bgr8")
            msg.header.stamp = data.header.stamp
            self.debugDetectionImagePub.publish(msg)

def main(args):
    "main function"
    rospy.init_node('track_vision')
    ld = TrackVision()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logwarn("Shutting down")

if __name__ == '__main__':
    main(sys.argv)