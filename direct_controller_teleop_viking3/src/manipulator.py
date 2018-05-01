#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Joy

# Utils
import math
from comms import can_handler

# msg ID is 0x200 for manipulator commands
ID = 0x200
# link select, rise MSB, rise LSB, sway MSB, sway LSB
values = [0, 0, 0, 0, 0]
old_vals = [0, 0, 0, 0, 0]


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	global ID
	global values
	global old_vals
	# fetch joypad controller input
	link_select = 0 	# TODO
	sway = -data.axes[2]
	rise = data.axes[3]
	if rise < 0:
		rise = int(rise * ((2**16-1) / 2) - 1)
	else:
		rise = int(rise * ((2**16-1) / 2))
	if sway < 0:
		sway = int(sway * ((2**16-1) / 2) - 1)
	else:
		sway = int(sway * ((2**16-1) / 2))
	r1, r2 = can_handler.split_bytes(rise, 2)
	s1, s2 = can_handler.split_bytes(sway, 2)
	values = [link_select, r1, r2, s1, s2]
	# send manipulator commands to CAN bus
	if values != old_vals:
		old_vals = values[:]
		can_handler.send_msg(ID, values)
	# send commands to CAN bus
	# can_handler.send_msg(ID, values)


def manipulator_control():
	# initialize ROS node 'manipulator'
	rospy.init_node('manipulator')
	# subscribe to the ROS topic 'joy'
	rospy.Subscriber('joy', Joy, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	manipulator_control()
