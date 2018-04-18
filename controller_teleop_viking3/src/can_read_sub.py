#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

import json


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	status = json.loads(data.data)
	print('\nCAN bus status:')
	for k in sorted(status):
		print('\t%s:\t%s' % (hex(int(k)), status[k]))


def can_read_sub():
	# initialize ROS node can_read_subscriber
	rospy.init_node('can_read_subscriber')
	# subscribe to the ROS topic CAN_statuses
	rospy.Subscriber('CAN_statuses', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	can_read_sub()
