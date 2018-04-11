#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Joy

# Utils
import math
from comms import can_handler

# msg ID is 0x050 for power commands
ID = 0x050
# power values
values = [0, 0]
old_vals = [0, 0]


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	# rospy.loginfo(data) # debug

	# send commands to CAN bus
	# can_handler.send_msg(ID, values)


def power_control():
	# initialize ROS node 'power'
	rospy.init_node('power')
	# subscribe to the ROS topic 'joy'
	rospy.Subscriber('joy', Joy, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	power_control()
