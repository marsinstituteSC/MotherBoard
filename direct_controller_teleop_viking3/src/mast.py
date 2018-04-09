#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Joy

# Utils
import math
from comms import can_handler


# msg ID is 0x150 for mast commands
ID = 0x150

values = [0, 0, 0, 0]


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	# rospy.loginfo(data) # debug

	# send commands to CAN bus
	# can_handler.send_msg(ID, values)


def mast_control():
	# initialize ROS node 'mast'
	rospy.init_node('mast')
	# subscribe to the ROS topic 'joy'
	rospy.Subscriber('joy', Joy, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	mast_control()
