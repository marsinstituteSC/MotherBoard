#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import json
from comms import can_handler

# msg ID is 0x150 for mast commands
ID = 0x150
# camera mast movement - left/right, down/up
values = [0, 0]
old_vals = [0, 0]


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	global ID
	global values
	global old_vals
	# fetch data from joy_events
	data = json.loads(data.data)
	# fetch joypad controller input
	pan = data['Axes']['6']		# pan left/right
	tilt = -data['Axes']['7']	# tilt down/up
	# update values
	if pan == -1:
		values = [0xFF, 0] 	# [-1, 0]: pan left
	elif pan == 1:
		values = [0x01, 0] 	# [1, 0]: pan right
	elif tilt == -1:
		values = [0, 0xFF] 	# [0, -1]: tilt down
	elif tilt == 1:
		values = [0, 0x01] 	# [0, 1]: tilt up
	else:
		values = [0, 0] 	# [0, 0]: no movement
	# send mast commands to CAN bus
	if values != old_vals:
		old_vals = values[:]
		can_handler.send_msg(ID, values)


def mast_control():
	# initialize ROS node 'mast'
	rospy.init_node('mast')
	# subscribe to the ROS topic 'joy_events'
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	mast_control()
