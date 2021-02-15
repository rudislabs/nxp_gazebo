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
        self.bridge = CvBridge()
        self.pyr_down = int(rospy.get_param("~pyr_down", 1))
        self.imgWHBoxRat = np.fromstring(rospy.get_param("~rwh", "0.4,0.45"), sep=',').astype(float)
        self.debug = rospy.get_param("~vision_debug", False)
        self.image_sub = rospy.Subscriber(
            "/nxp_cupcar/Pixy2CMUcam5/image_raw", sensor_msgs.msg.Image, self.callback)
        self.direction_debug_image_pub = rospy.Publisher(
            "direction_debug", sensor_msgs.msg.Image, queue_size=0)
        self.PixyVector_pub = rospy.Publisher(
            "PixyVector",nxp_gazebo.msg.PixyVector,queue_size=0)
        self.timeStart = time.time()
        self.steeringDeg = 0.0
        self.yCompRatio = .8
        self.borderDirectionRotDegCost = .25
        self.frameCount = 0
        self.oldMethod = False
       
        
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
        if self.debug:
            img3 = cv2.rectangle(img3,mask_box_top_left,mask_box_bottom_right,color=(128,128,0),thickness=3)
            dbgTopLeftXY = (int((iw/2)-(72/2)),int((ih/2)-(52/2)))
            img3 = cv2.rectangle(img3,dbgTopLeftXY,((dbgTopLeftXY[0]+72),(dbgTopLeftXY[1]+52)),(0,255,0),-1)
        maxcnt = 2
        if len (cnts) < maxcnt:
            maxcnt = len(cnts)
        cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0:maxcnt]

        if self.oldMethod:
            lineCenters = []
            lineRotations = []
        
        for cn in cnt:
            [vx,vy,lx,ly] = cv2.fitLine(cn, cv2.DIST_L2,0,0.01,0.01)
            tX = int((-ly*vx/vy)+lx)
            bX = int(((ih-ly)*vx/vy)+lx)
            lY = 0
            rY = ih
            if tX < 0:
                lY = int((-lx*vy/vx)+ly)
                tX = 0
            if tX > iw:
                lY = int((-lx*vy/vx)+ly)
                tX = 0
            if bX < 0:
                lY = int(((iw-lx)*vy/vx)+ly)
                bX = 0
            if bX > iw:
                lY = int(((iw-lx)*vy/vx)+ly)
                bX = 0

            tXSc = int(tX*(72/iw))
            bXSc = int(bX*(72/iw))
            lYSc = int(lY*(72/iw))
            rYSc = int(rY*(72/iw))

            timeStampMicroSeconds = (time.time()-self.timeStart)*1000000
            PixyVector_msg = nxp_gazebo.msg.PixyVector()
            PixyVector_msg.timestamp = int(timeStampMicroSeconds)
            PixyVector_msg.m_x0 = tXSc
            PixyVector_msg.m_y0 = lYSc
            PixyVector_msg.m_x1 = bXSc
            PixyVector_msg.m_y1 = rYSc
            PixyVector_msg.frame_count = self.frameCount
            self.PixyVector_pub.publish(PixyVector_msg)


            
            if self.debug:
                cv2.fillPoly(img3,pts=[cn],color=(0,0,255))
                img3 = cv2.line(img3,(tX,lY),(bX,rY),(255,128,128),2)
                img3 = cv2.line(img3,(dbgTopLeftXY[0]+tXSc,dbgTopLeftXY[1]+lYSc),(dbgTopLeftXY[0]+bXSc,dbgTopLeftXY[1]+rYSc),(255,128,128),1)
                img3 = cv2.rectangle(img3,(dbgTopLeftXY[0]+tXSc-1, dbgTopLeftXY[1]+lYSc-1),(dbgTopLeftXY[0]+tXSc+1, dbgTopLeftXY[1]+lYSc+1),(0,0,255),-1)
                img3 = cv2.rectangle(img3,(dbgTopLeftXY[0]+bXSc-1, dbgTopLeftXY[1]+rYSc-1),(dbgTopLeftXY[0]+bXSc+1, dbgTopLeftXY[1]+rYSc+1),(0,0,255),-1)
                img3 = cv2.putText(img3, '{:d},{:d}'.format(tXSc,lYSc), (dbgTopLeftXY[0]+tXSc, dbgTopLeftXY[1]+lYSc), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255,0,0), 1, cv2.LINE_AA)
                img3 = cv2.putText(img3, '{:d},{:d}'.format(bXSc,rYSc), (dbgTopLeftXY[0]+bXSc, dbgTopLeftXY[1]+rYSc), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255,0,0), 1, cv2.LINE_AA)
                
            if self.oldMethod:
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
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(img3,[box],0,(0,255,0),2)
                    cv2.putText(img3, '{:2.2f} Deg'.format(rotDeg), (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv2.LINE_AA)

        if self.frameCount < 255:
            self.frameCount += 1
        else:
            self.frameCount = 0

        if self.oldMethod:
            if len(lineRotations) != 2:
                rospy.logwarn("WARNING FOUND LESS THAN 2 LINE CONTOURS!!!")
            if len(lineRotations) == 2:
                borderDirectionRotDeg=lineRotations[0]+lineRotations[1]
                diffCenter=(int((lineCenters[0][0]+lineCenters[1][0])/2.0),int((lineCenters[0][1]+lineCenters[0][1])/2.0))
                transRotDeg = np.rad2deg(np.arctan2((diffCenter[0]-(iw/2.0)),((ih*self.yCompRatio)-diffCenter[1])))
                costRotDeg = transRotDeg+(self.borderDirectionRotDegCost*borderDirectionRotDeg)
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
        debug_img_direction = self.findDirection(img_direction)
        if self.debug:
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