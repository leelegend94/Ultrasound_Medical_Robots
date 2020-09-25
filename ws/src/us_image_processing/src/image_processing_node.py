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
from flownet2 import models

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class FlowNet2Args():
    fp16 = False
    rgb_max = 255

class ImageBuffer:
    def __init__(self,transform):
        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber("us_image",Image,self.update_image)
        self.pub_img = rospy.Publisher("mask",Image)
        self.transform = transform
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

def get_next_mask(label,flow):
    #print(flow.shape)
    #print(label.shape)
    label = np.array(label)
    result = np.zeros_like(label)
    
    try:
        rows,cols = np.where(label>0.5)
    except:
        rospy.loginfo(np.max(label))
        rospy.loginfo(np.where(label>0.5))
        return -1
    #rospy.loginfo(len(rows))

    for i in range(len(cols)):
        
        [dcol,drow] = flow[:,rows[i],cols[i]]
        
        tmp_row = np.clip(int(rows[i]+drow),0,label.shape[0]-1)
        tmp_col = np.clip(int(cols[i]+dcol),0,label.shape[1]-1)
        
        result[tmp_row,tmp_col] = 1
    
    return result

if __name__ == '__main__':
    rospy.init_node('image_segmentation')

    pub_pc2 = rospy.Publisher("us_vessel_pointcloud",PointCloud2)

    #init networks
    rospy.loginfo("loading UNet")
    PATH = os.path.expanduser("~/workspace/us_robot/network/unet_of_usseg_phantom.pth")
    unet = UNet_OF2(init_features=64).to(device)
    unet.load_state_dict(torch.load(PATH))
    unet.eval()

    rospy.loginfo("loading FlowNet2")
    args = FlowNet2Args()
    flownet = models.FlowNet2(args).cuda()
    flownet.load_state_dict(torch.load(os.path.expanduser('~/Downloads/FlowNet2_checkpoint.pth.tar'))["state_dict"])
    flownet.eval()

    rospy.loginfo("Initialization...")
    #resize_to=[256,256]
    transform_image = transforms.Compose([
        #transforms.Resize(resize_to),
        transforms.ToTensor(),
        transforms.Normalize(0.5,0.5)
    ])

    img_buf = ImageBuffer(transform=transform_image)
    
    while img_buf.get_image() is -1:
        rospy.sleep(0.2)

    img_init = img_buf.get_image()
    # rospy.loginfo("wtf")
    # rospy.loginfo(type(img_init))
    img_init_cu = img_init.to(device)

    with torch.no_grad():
        pred_cu = unet(img_init_cu, None, mode=0) #None? does it work? Not sure

    prev_label = pred_cu.cpu()
    prev_img_rgb = img_init.repeat(1,3,1,1,1) #(N,c,n,w,h)
    
    #TODO: init mask pre-processing

    sx = rospy.get_param('/calibration/scaling_x',1.4648e-4)
    sy = rospy.get_param('/calibration/scaling_y',1.5625e-4)
    cx = rospy.get_param('/calibration/c_x', -0.01875)
    cz = rospy.get_param('/calibration/cz', 0)
    # sx = 0.2/256
    # sy = 0.2/256
    # cx = -0.1
    # cz = 0
    
    calibMtx = np.array([[sx,0,cx],[0,0,0],[0,sy,cz]])

    while not rospy.is_shutdown():
        img = img_buf.get_image()
        img_rgb = img.repeat(1,3,1,1,1)

        im_cu = torch.cat((prev_img_rgb, img_rgb),2).to(device)

        with torch.no_grad():
            flow_cu = flownet(im_cu)
        #rospy.loginfo("max: %.3f" %(torch.max(flow_cu).item()))
        #rospy.loginfo("min: %.3f" %(torch.min(flow_cu).item()))
        #rospy.loginfo(flow_cu.shape)
        labels_curr_of = np.zeros_like(prev_label)
        flow = np.array(flow_cu.cpu())

        img_cu = img.to(device)
        #rospy.loginfo(img.shape)
        result = get_next_mask(prev_label[0,0,...],flow[0,...])
        
        if result is -1:
            rospy.loginfo("mask failed")
            with torch.no_grad():
                pred_cu = unet(img_cu, None, mode=0)
        else:
            #rospy.loginfo("good")
            labels_curr_of[0,...] = result
            labels_curr_of_cu = torch.Tensor(labels_curr_of).to(device)
            with torch.no_grad():
                pred_cu = unet(img_cu, labels_curr_of_cu, mode=1)
                #pred_cu = unet(img_cu, None, mode=0)
        
        pred = pred_cu.cpu()
        prev_img_rgb = img_rgb.clone()
        prev_label = np.copy(np.array(pred))

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
        rospy.loginfo(len(msg_pc2.data))
        pub_pc2.publish(msg_pc2)
