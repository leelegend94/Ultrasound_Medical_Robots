#!/usr/bin/env python
import rospy
import torch 
import torchvision.transforms as transforms

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField

import numpy as np
import os,sys

sys.path.append(os.path.expanduser("~/workspace/us_robot/network"))

from modules.UNet import *

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class ImageBuffer:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber("us_image",Image,self.update_image)
        self.pub_img = rospy.Publisher("mask",Image)
        self.img = None

    def update_image(self,msg):
        tmp = cv2.resize(self.bridge.imgmsg_to_cv2(msg,desired_encoding='mono8'),(256,256),interpolation=cv2.INTER_LANCZOS4)
        self.img = torch.Tensor((tmp.astype(np.float)/255-0.5)*2).unsqueeze(0).unsqueeze(0) #batch + color

    def get_image(self):
        if self.img is None:
            rospy.loginfo("Waiting for the first image.")
            return -1
        else:
            return self.img
            #TODO: return time stamp to avoid inaccurate tf

    def send_image(self,img):
        msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
        self.pub_img.publish(msg)

if __name__ == '__main__':
    rospy.init_node('image_segmentation')

    pub_pc2 = rospy.Publisher("us_vessel_pointcloud", PointCloud2, queue_size=10)

    #init networks
    rospy.loginfo("loading UNet")
    PATH = os.path.expanduser("~/workspace/us_robot/network/unet_of_usseg_phantom.pth")
    unet = UNet(init_features=64).to(device)
    unet.load_state_dict(torch.load(PATH))
    unet.eval()

    rospy.loginfo("Initialization...")
    #resize_to=[256,256]

    img_buf = ImageBuffer()
    
    while img_buf.get_image() is -1:
        rospy.sleep(0.2)

    sx = rospy.get_param('/calibration/scaling_x',1.4648e-4)
    sy = rospy.get_param('/calibration/scaling_y',1.5625e-4)
    cx = rospy.get_param('/calibration/c_x', -0.01875)
    cz = rospy.get_param('/calibration/cz', 0)

    calibMtx = np.array([[sx,0,cx],[0,0,0],[0,sy,cz]])

    while not rospy.is_shutdown():
        img = img_buf.get_image()

        img_cu = img.to(device)
        
        with torch.no_grad():
                pred_cu = unet(img_cu)
        
        pred = pred_cu.cpu()
        pred = np.array(pred[0].permute(1, 2, 0))
        pred = (pred*255).astype(np.uint8)
        _,pred = cv2.threshold(pred,thresh=127,maxval=255,type=cv2.THRESH_BINARY)
        img_buf.send_image(pred)
        #rospy.loginfo(pred.shape)

        #contour extraction
        try:
            _,contours,_ = cv2.findContours(pred, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        except:
            #rospy.loginfo("wtf")
            continue
        #to point cloud msg
        if len(contours) > 0:
            edge_points = np.array(contours[0].reshape([-1,2])).astype(np.float).transpose()
            edge_points = np.concatenate((edge_points, np.ones([1,edge_points.shape[1]])), axis=0)
            #rospy.loginfo(edge_points.shape)
            edge_points_3d = np.matmul(calibMtx,edge_points)
            data_array = edge_points_3d.transpose().reshape([-1]).astype(np.float32)
            #rospy.loginfo(len(data_array))
            msg_pc2 = PointCloud2()
            msg_pc2.header.frame_id = "cephalinear_link_ee"
            #msg_pc2.header.frame_id = "world"
            msg_pc2.header.stamp = rospy.Time.now()
            msg_pc2.height = 1
            msg_pc2.width = edge_points_3d.shape[1]
            msg_pc2.is_bigendian = False
            msg_pc2.point_step = 12 # 3(x,y,z)*4(byte)
            msg_pc2.row_step = edge_points_3d.shape[1]*12
            msg_pc2.is_dense = False

            msg_pc2.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                            PointField('y', 4, PointField.FLOAT32, 1),
                            PointField('z', 8, PointField.FLOAT32, 1)]

            msg_pc2.data = data_array.tostring()
            #rospy.loginfo(len(msg_pc2.data)," points are published.")
            pub_pc2.publish(msg_pc2)
        else:
            rospy.loginfo("lost target, point cloud is not published!")
