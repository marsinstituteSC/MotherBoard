#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import json
import math
import threading
from comms import can_handler

# mutex lock
mutex = threading.Lock()

# CAN bus read buffer: timestamp and data.
received = {}

# msg ID is 0x150 for mast commands
ID = 0x150

# camera mast tilt - left/right, down/up
values = [0, 0]
old_vals = [0, 0]


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	global mutex
	global received
	global ID
	global values
	global old_vals

	# fetch data from joy_events
	data = json.loads(data.data)
	# read CAN bus
	t = threading.Thread(target=can_handler.check_status, args=(ID, mutex, received)) # TODO: change to camera mast node status messages
	t.start()

	# fetch joypad controller input
	left_right = data['Axes']['6']	# tilt left/right
	up_down = -data['Axes']['7']	# tilt down/up

	if left_right == -1:
		values = [0xFF, 0] 	# [-1, 0]: tilt left
	elif left_right == 1:
		values = [0x01, 0] 	# [1, 0]: tilt right
	elif up_down == -1:
		values = [0, 0xFF] 	# [0, -1]: tilt down
	elif up_down == 1:
		values = [0, 0x01] 	# [0, 1]: tilt up
	else:
		values = [0, 0] 	# [0, 0]: no tilt

	# check CAN bus for relevant messages
	with mutex:
		try:
			# check latest relevant msg from drill node
			latest = received[max(received.keys())]
		except ValueError:
			latest = []

		if latest == values:
			# clear buffer and skip transmission if nodes already have current values
			received = {}
			pass
		else:
			# send drill commands to CAN bus
			if values == old_vals:
				pass
			else:
				old_vals = values[:]
				can_handler.send_msg(ID, values)


def mast_control():
	# initialize ROS node 'mast'
	rospy.init_node('mast')
	# subscribe to the ROS topic 'joy_events'
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	mast_control()
