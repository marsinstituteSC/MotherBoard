#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import json
import math
from comms import can_handler

# msg ID is 0x100 for wheel commands
ID = 0x100
# speed MSB, speed LSB,
# turn radius B1, turn radius B2, turn radius B3, turn radius B4
values = [0, 0, 0x80, 0, 0, 0]
old_vals = [0, 0, 0x80, 0, 0, 0]


def callback(data):
	"""
	param data:	data from joy_events topic
	"""
	global ID
	global values
	global old_vals
	# fetch data from joy_events
	data = json.loads(data.data)
	# fetch joypad controller input
	turning = data['Axes']['0']
	speed = data['Axes']['1']
	# update values
	# speed:
	s_MSB, s_LSB = can_handler.split_bytes(speed, 2)
	values[0] = int(s_MSB)
	values[1] = int(s_LSB)
	# turning:
	if turning == 0:
		# no turn, "infinite" turn radius (0x80000000 = -2**32 / 2)
		values[2] = 0x80
		values[3] = 0
		values[4] = 0
		values[5] = 0
	else:
		# using 32-bit value for turning radius
		if turning < 0:
			raw_tr = turning * (2**16)
			tr = (-2**32-1)/2 + 1 - raw_tr		# reverse axis
			tr = -int(math.sqrt(math.fabs(tr))) # reduce sensitivity
			if tr == 0:
				tr = -1	# turn left on own axis
		else:
			raw_tr = (turning + 1) * (2**16) - 1
			tr = (2**32-1)/2 - raw_tr			# reverse axis
			tr = int(math.sqrt(math.fabs(tr))) 	# reduce sensitivity
			if tr == 0:
				tr = 1	# turn right on own axis
		tr_B1, tr_B2, tr_B3, tr_B4 = can_handler.split_bytes(tr, 4)
		values[2] = int(tr_B1)
		values[3] = int(tr_B2)
		values[4] = int(tr_B3)
		values[5] = int(tr_B4)
	# send wheel commands to CAN bus
	if values != old_vals:
		old_vals = values[:]
		can_handler.send_msg(ID, values)


def wheel_control():
	# initialize ROS node 'wheels'
	rospy.init_node('wheels')
	# subscribe to the ROS topic 'joy_events'
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	wheel_control()
