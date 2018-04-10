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

# msg ID is 0x200 for manipulator commands
ID = 0x200

values = [0, 0, 0, 0]
old_vals = [0, 0, 0, 0]


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
	t = threading.Thread(target=can_handler.check_status, args=(ID, mutex, received)) # TODO: change to manipulator node status messages
	t.start()

	# print('Controller axes:', data['Axes']) 	    # debug
	# print('Controller buttons:', data['Buttons'])	# debug

	# check CAN bus for relevant messages
	with mutex:
		try:
			latest = received[max(received.keys())]	# check latest relevant msg from drill node
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


def manipulator_control():
	# initialize ROS node 'manipulator'
	rospy.init_node('manipulator')
	# subscribe to the ROS topic 'joy_events'
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	manipulator_control()
