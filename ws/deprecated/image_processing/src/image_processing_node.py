#!/usr/bin/env python
import rospy
import torch 
from net import UNet

from sensor_msgs.msg import Image

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class ImageProcessing:
	def __init__(self):
		self.model = UNet(init_features=32)
		self.model.load_state_dict(torch.load('/home/zhenyuli/workspace/us_robot/unet_usseg.pth')).to(device)
		self.sub_image = rospy.Subscriber("/us_image",Image,self.cb_unet_inference)
		self.pub_mask = rospy.Publisher("/us_segment",Image,queue_size=10)


	def cb_unet_inference(self,msg):
		rospy.loginfo("liuchang of course i still love you")
		msg.data


if __name__ == '__main__':
	rospy.init_node('test')
	rospy.loginfo("liuchang i love you")
	ImageProcessing()
	rospy.spin()