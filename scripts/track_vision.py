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
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
if cv2.__version__ < "4.0.0":
    raise ImportError("Requires opencv >= 4.0, "
                      "but found {:s}".format(cv2.__version__))
class TrackVision(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.pyr_down = int(rospy.get_param("~pyr_down", 1))
        self.imgWHBoxRat = np.fromstring(rospy.get_param("~rwh", "0.4,0.45"), sep=',').astype(float)
        self.debug = rospy.get_param("~vision_debug", False)
        self.image_sub = rospy.Subscriber(
            "/nxp_cupcar/Pixy2CMUcam5/image_raw", sensor_msgs.msg.Image, self.callback)
        self.direction_debug_image_pub = rospy.Publisher(
            "direction_debug", sensor_msgs.msg.Image, queue_size=0)
        self.steeringDeg = 0.0
        self.yCompRatio = .8
        self.borderDirectionRotDegCost = .25
       
        
    def findDirection(self, momentImage):
        imgc = cv2.cvtColor(momentImage,cv2.COLOR_BGR2GRAY)
        ih, iw = imgc.shape[:2]
        thresh = cv2.bitwise_not(cv2.threshold(imgc, 10, 255, cv2.THRESH_BINARY)[1])
        mask = np.ones(thresh.shape[:2], dtype="uint8") * 255
        mask_box_top_left = (int(iw*(1.0-self.imgWHBoxRat[0])/2.0),int(ih*(1.0-self.imgWHBoxRat[1])))
        mask_box_bottom_right = (int(iw*(1.0+self.imgWHBoxRat[0])/2.0),int(ih))
        mask = cv2.rectangle(mask,mask_box_top_left,mask_box_bottom_right,color=0,thickness=-1)
        thresh = cv2.bitwise_and(thresh, thresh, mask=mask)
        cnts, hierarchy = cv2.findContours(thresh.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        img3=momentImage
        maxcnt = 2
        if len (cnts) < maxcnt:
            maxcnt = len(cnts)
        cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0:maxcnt]

        lineCenters = []
        lineRotations = []
        for cn in cnt:
            rect = cv2.minAreaRect(cn)
            #https://namkeenman.wordpress.com/2015/12/18/open-cv-determine-angle-of-rotatedrect-minarearect/
            if rect[1][0] < rect[1][1]:
                rotDeg180 = rect[2]-90
            else:
                rotDeg180 = rect[2]
            rotDeg = rotDeg180+90
            (cX, cY) = (int(rect[0][0]),int(rect[0][1]))
            lineCenters.append([cX,cY])
            lineRotations.append(rotDeg)
    
            if self.debug:
                cv2.fillPoly(img3,pts=[cn],color=(0,0,255))
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(img3,[box],0,(0,255,0),2)
                cv2.putText(img3, '{:2.2f} Deg'.format(rotDeg), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv2.LINE_AA)

        if len(lineRotations) != 2:
            rospy.logwarn("WARNING FOUND LESS THAN 2 LINE CONTOURS!!!")
        if len(lineRotations) == 2:
            borderDirectionRotDeg=lineRotations[0]+lineRotations[1]
            diffCenter=(int((lineCenters[0][0]+lineCenters[1][0])/2.0),int((lineCenters[0][1]+lineCenters[0][1])/2.0))
            transRotDeg = np.rad2deg(np.arctan2((diffCenter[0]-(iw/2.0)),((ih*self.yCompRatio)-diffCenter[1])))
            costRotDeg = transRotDeg+(self.borderDirectionRotDegCost*borderDirectionRotDeg)
    
    
            if self.debug:
                img3 = cv2.rectangle(img3,mask_box_top_left,mask_box_bottom_right,color=(128,128,0),thickness=3)
                R=np.linalg.norm(np.array(((iw/2.0),(ih*self.yCompRatio)))-np.array(diffCenter))
                deX=diffCenter[0]+int(0.25*R*np.cos(np.deg2rad(90.0-borderDirectionRotDeg)))
                deY=diffCenter[1]-int(0.25*R*np.sin(np.deg2rad(90.0-borderDirectionRotDeg)))
                ceX=int((iw/2.0)+1.25*R*np.cos(np.deg2rad(90.0-costRotDeg)))
                ceY=int(ih-1.25*R*np.sin(np.deg2rad(90.0-costRotDeg)))
        
                cv2.arrowedLine(img3,diffCenter,(deX,deY),(255,0,0),4,tipLength = 0.2)
                cv2.arrowedLine(img3,(int(iw/2.0),int(ih*self.yCompRatio)),diffCenter,(0,255,0),4,tipLength = 0.2)
                cv2.arrowedLine(img3,(int(iw/2.0),int(ih)),(ceX,ceY),(0,0,255),10,tipLength = 0.2)
        
                strborderDirectionRotDeg='{:2.2f} Deg'.format(borderDirectionRotDeg)
                strTransRotDeg='{:2.2f} Deg'.format(transRotDeg)
                strCostRotDeg='{:2.2f} Deg'.format(costRotDeg)
        
                cv2.putText(img3, strborderDirectionRotDeg, diffCenter, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128,0,0), 2, cv2.LINE_AA)
                cv2.putText(img3, strTransRotDeg, (int(iw/2.0)+10,int(ih*self.yCompRatio)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,128,0), 2, cv2.LINE_AA)
                cv2.putText(img3, strCostRotDeg, (int(iw/2.0)+10,int(ih*0.99)), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,128), 2, cv2.LINE_AA)
            self.steeringDeg = costRotDeg
        return(img3)
      
    
    def callback(self, data):
        scene = self.bridge.imgmsg_to_cv2(data, "bgr8")
        scene_direction = copy.deepcopy(scene)
        if self.pyr_down > 0:
            for i in range(self.pyr_down):
                scene_direction = cv2.pyrDown(scene_direction)
        img_direction = copy.deepcopy(scene_direction)        
        img_direction = self.findDirection(img_direction)
        msg = self.bridge.cv2_to_imgmsg(img_direction, "bgr8")
        msg.header.stamp = data.header.stamp
        self.direction_debug_image_pub.publish(msg)

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