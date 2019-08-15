import sys
sys.path.insert(0,'/home/srss9523/.conda/envs/mask/lib/python3.6/site-packages')
import numpy as np
import cv2
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
import threading
import time
from fishdata.msg import data_processing
from geometry_msgs.msg import PoseArray 
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
from math import atan2, cos, sin, sqrt, pi
import copy
import tensorflow as tf
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

import os,sys,cv2
ROOT_DIR = os.path.abspath("/home/srss9523/wendang/Mask_RCNN")

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
from mrcnn import utils
from mrcnn import visualize
from mrcnn.visualize import display_images
import mrcnn.model as modellib
from mrcnn.model import log
import sys

DEVICE = "/gpu:0"  # /cpu:0 or /gpu:0
TEST_MODE = "inference"
BALLON_WEIGHTS_PATH ="/home/srss9523/catkin_ws/src/swarmplatform/src/mask_rcnn_shapes_0040.h5"
from mrcnn.config import Config
MODEL_DIR = "/home/srss9523/catkin_ws/src/swarmplatform/logs"
class BalloonConfig(Config):
    """Configuration for training on the toy  dataset.
    Derives from the base Config class and overrides some values.
    """
    # Give the configuration a recognizable name
    NAME = "fish"
    BATCH_SIZE=1
    # We use a GPU with 12GB memory, which can fit two images.
    # Adjust down if you use a smaller GPU.
    IMAGES_PER_GPU = 1
    BACKBONE = "resnet50"
    # Number of classes (including background)
    NUM_CLASSES = 1 + 1  # Background + balloon

    # Number of training steps per epoch
    STEPS_PER_EPOCH = 100
    PRE_NMS_LIMIT=300
    # Skip detections with < 90% confidence
    DETECTION_MIN_CONFIDENCE = 0.7
    DETECTION_MAX_INSTANCES=10
    IMAGE_MIN_DIM = 480
    IMAGE_MAX_DIM = 640
    POST_NMS_ROIS_INFERENCE = 20    
    # Use smaller anchors because our image and objects are small
    RPN_ANCHOR_SCALES = (8 * 6, 16 * 6, 32 * 6, 64 * 6, 128 * 6)  # anchor side in pixels








##=============================== function ===============================##
weights_path = BALLON_WEIGHTS_PATH
import os
class CV_process(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.config = BalloonConfig()
        self.weights_path = BALLON_WEIGHTS_PATH
        with tf.device(DEVICE):
            self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR,
                                      config=self.config)
            self.model.load_weights(weights_path, by_name=True)
        rospy.init_node('maskcv_processing')
        self.img_rgb = None
        self.img_depth = None
        self.result = 'waiting...'
        self.image_lock = threading.RLock()
        self.cX = np.zeros(9)
        self.cY = np.zeros(9)
        self.ii=1
#        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', numpy_msg(Image), self.update_image)
       # self.image_depth_sub = rospy.Subscriber('/camera_image', numpy_msg(Image), self.process)

        self.bridge = CvBridge()
        self.dirpic=os.listdir("fishpic")
        
        #self.image_depth_sub = rospy.Subscriber('/camera2d', Image, self.process)
        self.pub = rospy.Publisher('/pose_from_processing', data_processing, queue_size=1)
        
        

        

 
           # print re
    def getOrientation(self, ptx,pty):
        
        sz = len(ptx)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts [i,0] =ptx[i]
            data_pts[i,1] = pty[i]
        # Perform PCA analysis
        mean = np.empty((0))
        mean, eigenvectors = cv2.PCACompute(data_pts, mean=None)
        # Store the center of the object
        cntr = (int(mean[0,0]), int(mean[0,1]))
        
        
       
        p1 = (cntr[0] + 0.2 * eigenvectors[0,0] * 1, cntr[1] + 0.2 * eigenvectors[0,1] * 1)
      #  p2 = (cntr[0] - 0.2 * eigenvectors[1,0] * 1, cntr[1] - 0.2 * eigenvectors[1,1] * 1)

       # self.drawAxis(img, cntr, p2, (255, 255, 0), 120)
        angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
       # print(str(angle)+ "$$$$$"+ str(i))
        return angle

        


    def process(self):
    #def process(self,img_rgb):
        #print("i am called ")
        dd=np.random.randint(1,3,(4,5))
        dp=data_processing()
        self.ii=self.ii+1
        i=1
        #self.img_rgb=np.frombuffer(img_rgb.data,dtype=np.uint8).reshape(img_rgb.width,img_rgb.height,-1)
        self.img_rgb=cv2.imread("fishpic/"+str(i)+".png")

        #self.img_rgb = self.bridge.imgmsg_to_cv2(img_rgb, "bgr8")
        #self.img_rgbcpy=copy.deepcopy(self.img_rgb)
        while type(self.img_rgb) !=type(dd):
            #print(type(self.img_rgb))
            i=(i+1)%4
            self.img_rgb=cv2.imread("fishpic/"+str(i)+".png")

        #print(type(self.img_rgb)=="NoneType")
        #print(self.img_rgb)

	
        aa=time.clock()
        results = self.model.detect([self.img_rgb], verbose=0)
        print(time.clock()-aa,"tiem elapseingggggggggggggggggggg")
        r = results[0]
        rmask=r['masks']
        for i in range(np.shape(rmask)[2]):

            ptxx,ptyy=np.where(rmask[:,:,i])
            
            ff=self.getOrientation(ptyy,ptxx)
            xx=np.mean(ptxx)
            yy=np.mean(ptyy)
         #   print(xx,yy,"this is the center")
            dp.pose_x.append(yy)
            dp.pose_y.append(xx)                 
            dp.angle.append(ff)
           # cv2.circle(self.img_rgb, (int(xx), int(yy)), 4, (0, 255, 0), 2)
           # cv2.imshow('drawimg',self.img_rgb)
           # cv2.waitKey(5000)
        self.pub.publish(dp)
        #print("i am called 3")

if __name__ == '__main__':

    node = CV_process()
    while not rospy.is_shutdown():
        node.process()
#        if node.img_rgb is not None:
    
          #  pass
