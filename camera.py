#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri May 24 22:21:10 2019

@author: taosheng
"""

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jan  8 05:25:45 2019

@author: taosheng
"""
import sys

sys.path.insert(0,'/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import cv2
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import threading
import time

from geometry_msgs.msg import PoseArray 
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from math import atan2, cos, sin, sqrt, pi
import copy
#自定义的数据类型
from fishdata.msg import data_processing

import matplotlib.pyplot as plt
bridge = CvBridge()
k=0
##============================ label define ==============================##
#0sanlengzhui 1cube 2sanlengzhu 3changfangti 4yuanzhu 5liulengzhu 6silengtai 7silengzhui 8yuanzhui 9sphere
##========================================================================##

##=========================== hyper-parameters ===========================##
# area
AREA_THRED = 2000 # number used to distinguish big and small object
AREA_BIG = 2700
AREA_SAMLL = 1500
# perimeter
PERIMETER_THRED = 100
# camera parameters  1.3962634
CX = 321
CY = 237
FX = 381.3612
FY = 381.3612
factor = 0.01

target_array = [1,2,3,4,5,6,7,8,9,10]
#target_array = [1,4,5,9,10].append











object_paremeter = [[0.05, 3500, 6000],  # 1sanlengzhui
                   [0.064, 3600, 5000],  # 2cube
                   [0.056, 2500, 5500],  # 3sanlengzhu
                   [0.07, 3000, 5800],   # 4changfangti
                   [0.07, 3300, 5600],   # 5yuanzhu
                   [0.068, 3000, 5500],  # 6liulengzhu
                   [0.062, 3000, 5500],  # 7silengtai
                   [0.068, 3300, 5000],  # 8silengzhui
                   [0.07, 3000, 4800],   # 9yuanzhui
                   [0.07, 3600, 4800]]   # 10sphere

##=============================== function ===============================##


class KinectNode(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.pubcam = rospy.Publisher('/camera_image', Image, queue_size=1)
        rospy.init_node('camera_node')
        self.i=0
        self.img_rgb = None
        self.img_depth = None
        self.result = 'waiting...'
        self.processflag=False
        self.image_lock = threading.RLock()
        self.cX = np.zeros(9)
        self.cY = np.zeros(9)
        self.angle = np.zeros(9)
        #self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', numpy_msg(Image), self.update_image)
        self.bridge = CvBridge()
        self.cam=cv2.VideoCapture(0)
	#read()是双返回参数的函数
        re,self.img_rgb=self.cam.read()

	#新开一个线程，用来发布以及显示图片
        t = threading.Thread(target=self.update_rgbimage, args=())
        t.start()
	#向主节点注册话题postition_out，用来发布位置信息（data_processing是自定义的数据类型）
        self.pub = rospy.Publisher('/postition_out', data_processing, queue_size=1)
	#订阅pose_from_processing话题
        self.sub_process = rospy.Subscriber('/pose_from_processing', data_processing, self.process)
        self.data0=None
        self.data1=None
        self.roiPts=[]
        self.depth_image=None
        rate = rospy.Rate(2000)
        self.fishid=None
        self.hypo=[]
        


    def update_image(self, data):
        self.data0=data
        img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.img_rgb = img
        

    def update_rgbimage(self):
        while True:
            #print(np.where(self.cX),"this is non zero one")
            self.pubcam.publish(self.bridge.cv2_to_imgmsg(self.img_rgb, "bgr8"))
            if np.size(np.where(self.cX))!=0:
                for i in np.nditer(np.where(self.cX)):
                   # print (i)
                    cv2.circle(self.img_rgb, (int(self.cX[i]), int(self.cY[i])), 4, (0, 255, 0), 2)
                    cv2.putText(self.img_rgb, str(i+1),(int(self.cX[i]), int(self.cY[i])), cv2.FONT_HERSHEY_SIMPLEX, 1, 250)
                    self.drawAxis(self.img_rgb,[int(self.cX[i]), int(self.cY[i])],self.angle[i])
                    #print("done with axis")
            self.i=self.i+1
            cv2.namedWindow("imshow")
            cv2.imshow("imshow",self.img_rgb)
            cv2.setMouseCallback("imshow",self.click_and_crop)
            key = cv2.waitKey(50) & 0xFF
            re,self.img_rgb=self.cam.read()
            cv2.imwrite("fishpic/"+str(self.i%4)+".png",self.img_rgb)
        #print(key)

            if key >=49 and key <=57:
             #   print(key)
              #  print("i ma "+ str(key-48))
                self.fishid=key-48
            if key == 27:
                
                self.realease()
                break
           # print re

        
        
        
    def drawAxis(self,img, cent_pot, angle, colour=(0, 255, 0), scale=1):

        p=cent_pot
        
        hypotenuse = 30
        q=[0,0]
        #print(q,"this is cne point")
       # plt.show()
        #angle=filtedData[-1]
        
        q[0] = cent_pot[0] + scale * hypotenuse * cos(angle)
        q[1] = cent_pot[1] + scale * hypotenuse * sin(angle)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
        #cv2.circle(self.img_rgb, (int(q[0]), int(q[1])), 4, (0, 255, 0), 2)
        # create the arrow hooks
        p[0] = q[0] - 9 * cos(angle + pi / 4)
        p[1] = q[1] - 9 * sin(angle + pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
        p[0] = q[0] - 9 * cos(angle - pi / 4)
        p[1] = q[1] - 9 * sin(angle - pi / 4)
        cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)
       
    def update_depth_image(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
#        data=numpy_msg(data)
#        img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
#        self.img_depth = img
    def click_and_crop(self,event, x, y, flags, param):
            # grab the reference to the current frame, list of ROI
            # points and whether or not it is ROI selection mode
            
         
            # if we are in ROI selection mode, the mouse was clicked,
            # and we do not already have four points, then update the
            # list of ROI points with the (x, y) location of the click
            # and draw the circle
            
            if  event == cv2.EVENT_LBUTTONDOWN :
         #       print ("id  "+str(self.fishid))
                if self.fishid != None:
          #          print(x,y)
                    self.cX[self.fishid-1]=x
                    self.cY[self.fishid-1]=y
           #         print (self.cX)
               


            if  event == cv2.EVENT_LBUTTONUP :
                if self.fishid != None:
                    theta=atan2(y-self.cY[self.fishid-1],x-self.cX[self.fishid-1])
                    self.angle[self.fishid-1]=theta
                    self.fishid=None
                    


    def wrapangle(self,anglelist):
        for i in range(len(anglelist)):            
            if anglelist[i] >3.14159:
                anglelist[i]=anglelist[i]-2*3.14159
            if anglelist[i] <-3.14159:
                anglelist[i]=anglelist[i]+2*3.14159
        return anglelist
                
    def compa(self,anglereal,angle):
        gg=[angle,angle-3.14159/2,angle+3.14159/2,angle-3.14159]
        wraped_angle=self.wrapangle(gg)
        
        ind=np.argmin(np.square(np.array(anglereal-wraped_angle)))
        return wraped_angle[ind]
    def realease(self):
        
        cv2.destroyAllWindows()
        self.cam.release()        
        
	#回调函数
    def process(self,data):
        ps=data_processing()
        

     #   cv2.imwrite('2.jpg',im_binary)
      #  cv2.imwrite('11111.jpg',im_binary)

        
      #  contours = np.array(new_contours)
       
        object_number=9
       # print object_number
       # print("recognize object number is ", object_n36.0umber)

        cx=data.pose_x
        cy=data.pose_y
        #print(cx)1
        angle=data.angle
        self.img_rgbcpy=copy.deepcopy(self.img_rgb)
        # calculate all parameters
        for i in range(object_number):
#                if i<len(contours1):
#                    cnt = contours[i]
#                    M = cv2.moments(cnt)
#                    cx = int(M["m10"] / M["m00"])
#                    cy = int(M["m01"] / M["m00"])
#                else:
#                    cx = 0
#                    cy = 0
                
                # draw center of the shape on the image
                #cv2.circle(im_rgb, (int(cX[i]), int(cY[i])), 5, (0, 255, 0), -1)
                # calculate the position in camera frame
                #Z[i] = 0.98 #(img_depth[int(cX[i])][int(cY[i])][0]) * factor - 0.6
 
             #   print("cX,cY = ", cX[i],cY[i])2
                # find label

                #print cx
                #print self.cX
                gg=np.sqrt( np.square(cx-self.cX[i])+np.square(cy-self.cY[i]) )<100
                #print("gg======", gg)
               # print(self.cX)
                hh=np.where(gg)
               # print i
                #print("hh=====", hh) 
                
                if np.size(hh)>=1:
                    #print("zhaodao yige **************")
                    #print hh[0][0]
                   
                    self.cX[i]=cx[hh[0][0]]
                    self.cY[i]=cy[hh[0][0]]
            #        print(angle[hh[0][0]],"angle form process")
             #       print(self.angle)
                    self.angle[i]=self.compa(self.angle[i],angle[hh[0][0]])
              #      print(self.angle,"this is th angle")
                    ps.pose_x.append(cx[hh[0][0]])
                    ps.pose_y.append(cy[hh[0][0]])
                    ps.angle.append(self.angle[i])
               #     print("i am done ")
                    #print i
                    #self.getOrientation(new_contours[hh[0][0]],self.img_rgbcpy)
                else:
                    ps.pose_x.append(0)
                    ps.pose_y.append(0)
                    ps.angle.append(0)

                 	 
                
#        cv2.imshow("imshow2",self.img_rgbcpy)
#        key = cv2.waitKey(5000)& 0xFF
      #  print("are u ok?")
        self.pub.publish(ps)
       # print("are u ok2222?")
        
          #  print("area, perimeter", areas[i], perimeters[i])
            
            # judge label via area and perimeter
    
                  #  print(8)
       ##     cv2.circle(im1, (int(cX[i]),int(cY[i])), 5, (0,0,255), -1)
          
           # cv2.putText(im6,str(int(labels[i])), (int(int(cX[i])),int(cY[i])), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0))
                    #cv2.circle(im_rgb, (int(cX[i]), int(cY[i])), 2, (150, 255, 0), -1)
    
    
        # publish object
    #    pub_result = np.zeros([6,1])
    #   
    #    # pub_result =[[0 for col in range(9)] for row in range(7)]
    #    if len(new_contours) >0:
    #        nn=np.array(areas1)
    #        ii=nn.argmax()
    #       # nowTime = time.time()
    #        #print(nowTime)
    #        print(k)
    #        pub_result[0] =k
    #        
    #        pub_result[1] = labels[ii]
    #        pub_result[2] = X[ii]
    #        pub_result[3] = Y[ii]
    #        pub_result[4] = Z[ii]
    #        pub_result[5] = 0  # rotation
    #                    #pub_result[i][6:9] = object_paremeter[target_id]
        


if __name__ == '__main__':
    # img_rgb = cv2.imread('qqq.jpg')
    # img_depth = cv2.imread('www.jpg')
    # process(img_rgb,img_depth)
#    node = KinectNode()
#    while not rospy.is_shutdown():
#        if node.img_rgb is not None:
#            k=k+1
#            pub_result = node.process(node.img_rgb,k)  
#           # print(pub_result)
#            if pub_result is not None:
#               # print("Publish result is" + str(pub_result))
#                node.pub.publish(pub_result)
#            else:
#                print("No result to publish!")
#        else:
#            rospy.loginfo("No image received!")
    node = KinectNode()
    while not rospy.is_shutdown():
        if node.img_rgb is not None:
            pass
