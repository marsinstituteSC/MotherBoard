#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Joy

# Utils
import math
from comms import can_handler


# msg ID is 0x100 for wheel commands
ID = 0x100

# speed MSB, speed LSB,
# turn radius B1, turn radius B2, turn radius B3, turn radius B4
values = [0, 0, 255, 255, 255, 255]


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	# can_handler.check_status(ID)  # TODO
	# rospy.loginfo(data) # debug
	turning = data.axes[0]
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
		# no turn, infinite turn radius
		values[2] = 255
		values[3] = 255
		values[4] = 255
		values[5] = 255
	else:
		# using raw 32-bit value for turning radius
		if turning < 0:
			raw_tr = -int(turning * ((2**32-1) / 2))
		else:
			raw_tr = -int(turning * ((2**32-1) / 2) + 1)
		tr_B1, tr_B2, tr_B3, tr_B4 = can_handler.split_bytes(raw_tr, 4)
		values[2] = int(tr_B1)
		values[3] = int(tr_B2)
		values[4] = int(tr_B4)
		values[5] = int(tr_B4)
	# send wheel commands to CAN bus
	can_handler.send_msg(ID, values)


def wheel_control():
	# initialize ROS node 'wheels'
	rospy.init_node('wheels')
	# subscribe to the ROS topic 'joy'
	rospy.Subscriber('joy', Joy, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	wheel_control()
