#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import json

prev_status = None
new_status = None


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	global prev_status
	global new_status
	new_status = json.loads(data.data)
	if new_status != prev_status:
		# print updated CAN bus status
		print('\nCAN bus status:')
		for k in sorted(new_status):
			print('\t%s:\t%s' % (hex(int(k)), new_status[k]))
		prev_status = new_status


def can_read_sub():
	# initialize ROS node can_read_subscriber
	rospy.init_node('can_read_subscriber')
	# subscribe to the ROS topic CAN_statuses
	rospy.Subscriber('CAN_statuses', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	can_read_sub()
