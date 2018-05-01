#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Joy

# Utils
import math
from comms import can_handler

# msg ID is 0x300 for digger commands
ID = 0x300
# digger values
values = [0, 0]
old_vals = [0, 0]


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	global ID
	global values
	global old_vals
	# fetch joypad controller input and update values
	direction = data.buttons[6]
	digging = math.fabs(data.axes[5] - 1)
	values[0] = direction
	values[1] = int(digging * ((2**8-1) / 2 + 1))
	if values[1] > 255:
		values[1] = 255
	# send digger commands to CAN bus
	if values != old_vals:
		old_vals = values[:]
		can_handler.send_msg(ID, values)


def digger_control():
	# initialize ROS node 'digger'
	rospy.init_node('digger')
	# subscribe to the ROS topic 'joy'
	rospy.Subscriber('joy', Joy, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	digger_control()
