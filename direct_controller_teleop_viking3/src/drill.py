#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Joy

# Utils
import math
import threading
from comms import can_handler

# mutex lock
mutex = threading.Lock()

# CAN bus read buffer: timestamp and data.
received = {}

# msg ID is 0x250 for drill commands
ID = 0x250

# clockwise/counter-clockwise, drill speed
values = [0, 0]
old_vals = [0, 0]

# flag for calibration/bug fix
calib = False


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
	global calib

	# read CAN bus
	t = threading.Thread(target=can_handler.check_status, args=(ID, mutex, received)) # TODO: change to drill node status messages
	t.start()

	# TODO: fix bug
	if data.axes[4] != 0 and calib == False:
		calib = True

	# fetch joypad controller input
	drilling = math.fabs(data.axes[4] - 1)
	mode = data.buttons[7]

	if calib == True:
		if mode == 0:
			# clockwise
			values[0] = 0
			values[1] = int(drilling * ((2**8-1) / 2 + 1))
		else:
			# counter-clockwise
			values[0] = 1
			values[1] = int(drilling * ((2**8-1) / 2 + 1))
		if values[1] > 255:
			values[1] = 255

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


def drill_control():
	# initialize ROS node 'drill'
	rospy.init_node('drill')
	# subscribe to the ROS topic 'joy'
	rospy.Subscriber('joy', Joy, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	drill_control()
