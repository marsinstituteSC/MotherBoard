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

# msg ID is 0x300 for digger commands
ID = 0x300

values = [0, 0, 0, 0]
old_vals = [0, 0, 0, 0]


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	# rospy.loginfo(data) # debug
	global mutex
	global received
	global ID
	global values
	global old_vals

	data = json.loads(data.data)
	# print('Controller axes:', data['Axes']) 	    # debug
	# print('Controller buttons:', data['Buttons'])	# debug

	# read CAN bus
	t = threading.Thread(target=can_handler.check_status, args=(ID, mutex, received)) # TODO: change to digger node status messages
	t.start()

	# fetch joypad controller input
	# drilling = data['Axes']['5']

	# check CAN bus for relevant messages
	mutex.acquire()
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
	mutex.release()


def digger_control():
	# initialize ROS node 'digger'
	rospy.init_node('digger')
	# subscribe to the ROS topic 'joy_events'
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	digger_control()
