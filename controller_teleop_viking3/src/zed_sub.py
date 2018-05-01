#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Image

# Utils
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


def callback(data):
	"""
	param data:	data from zed/rgb/image_rect_color topic
	"""
	# convert from ROS sensor_msgs/Image to cv2
	bridge = CvBridge()
	try:
		cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
		cv2.imwrite('frame.jpeg', cv_img)
	except CvBridgeError as e:
		print(e)
	# show image stream on on-board computer:
	# cv2.imshow('zed', cv_img)
	# cv2.waitKey(3)


def zed_sub():
	# initialize ROS node 'zed_sub'
	rospy.init_node('zed_sub')
	# subscribe to the ROS topic 'zed/rgb/image_rect_color'
	rospy.Subscriber('zed/rgb/image_rect_color', Image, callback)
	# keep python from exiting until this node is stopped
	try:
		rospy.spin()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()


if __name__ == '__main__':
	zed_sub()
