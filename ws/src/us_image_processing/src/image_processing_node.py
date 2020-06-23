#!/usr/bin/env python
import rospy
import torch 

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

#device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

class PoseEstimator:
	def __init__(self):
		self.container = []
		self.bridge_ = CvBridge()
		self.sub_mask = rospy.Subscriber("/us_segment",Image,self.cb_update)


	def cb_update(self,msg):
		rospy.loginfo("update")
		mask = self.bridge_.imgmsg_to_cv2(msg,desired_encoding='passthrough')
		
		x,y = cv2.findNonZero(mask)
		for i in range(0,len(x),5):
			np.array([x,y,1])

		


if __name__ == '__main__':
	rospy.init_node('test')
	rospy.loginfo("liuchang i love you")
	ImageProcessing()
	rospy.spin()