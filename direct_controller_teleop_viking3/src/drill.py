#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Joy

# Utils
import math
from comms import can_handler

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
	global ID
	global values
	global old_vals
	global calib
	# TODO: fix bug:
	if data.axes[4] != 0 and calib == False:
		calib = True
	# fetch joypad controller input
	drilling = math.fabs(data.axes[4] - 1)
	mode = data.buttons[7]
	# update values
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
	# send drill commands to CAN bus
	if values != old_vals:
		old_vals = values[:]
		can_handler.send_msg(ID, values)


def drill_control():
	# initialize ROS node 'drill'
	rospy.init_node('drill')
	# subscribe to the ROS topic 'joy'
	rospy.Subscriber('joy', Joy, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	drill_control()
