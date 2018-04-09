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

# msg ID is 0x100 for wheel commands
ID = 0x100

# Values to be sent onto CAN bus. Speed MSB, speed LSB,
# turn radius B1, turn radius B2, turn radius B3, turn radius B4
values = [0, 0, 0, 0, 0, 0]
old_vals = [0, 0, 0, 0, 0, 0]


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

	# read CAN bus
	t = threading.Thread(target=can_handler.check_status, args=(ID, mutex, received)) # TODO: change to wheel node status messages
	t.start()

	# fetch joypad controller input
	turning = data.axes[0]**3	# power of 3 to reduce sensitivity
	speed = data.axes[1]

	# convert raw data to uniform values wrt. linux character device values.
	# ROS joy gives float values between -1 and 1
	# speed:
	if speed < 0:
		raw_s = int(speed * ((2**16-1) / 2) - 1)
	else:
		raw_s = int(speed * ((2**16-1) / 2))
	s_MSB, s_LSB = can_handler.split_bytes(raw_s, 2)
	values[0] = int(s_MSB)
	values[1] = int(s_LSB)
	# turning:
	if turning == 0:
		# no turn, no turn radius
		values[2] = 0
		values[3] = 0
		values[4] = 0
		values[5] = 0
	else:
		# using 32-bit value for turning radius
		if turning < 0:
			raw_tr = -turning * ((2**32-1) / 2)
			tr = (2**32-1)/2 - raw_tr	# reverse axis
			tr = int(math.sqrt(math.fabs(tr * (1 - math.fabs(turning))))) 	# reduce sensitivity
			if tr == 0:
				tr = 1	# turn right on own axis
		else:
			raw_tr = -turning * ((2**32-1) / 2) + 1
			tr = (-2**32-1)/2 - raw_tr	# reverse axis
			tr = -int(math.sqrt(math.fabs(tr * (1 - math.fabs(turning))))) 	# reduce sensitivity
			if tr == 0:
				tr = -1	# turn left on own axis
		tr_B1, tr_B2, tr_B3, tr_B4 = can_handler.split_bytes(tr, 4)
		values[2] = int(tr_B1)
		values[3] = int(tr_B2)
		values[4] = int(tr_B3)
		values[5] = int(tr_B4)

	# check CAN bus for relevant messages
	mutex.acquire()
	try:
		# check latest relevant msg from wheel nodes
		latest = received[max(received.keys())]
	except ValueError:
		latest = []

	if latest == values:
		# clear buffer and skip transmission if nodes already have current values
		received = {}
		pass
	else:
		# send wheel commands to CAN bus
		if values == old_vals:
			pass
		else:
			old_vals = values[:]
			can_handler.send_msg(ID, values)
	mutex.release()


def wheel_control():
	# initialize ROS node 'wheels'
	rospy.init_node('wheels')
	# subscribe to the ROS topic 'joy'
	rospy.Subscriber('joy', Joy, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	wheel_control()
