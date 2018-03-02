#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Joy

# Utils
import math
from comms import can_handler


# msg ID is 0x100 for wheel commands
ID = 0x100
# speed fwd, speed reverse, turn radius MSB, turn radius LSB, turn direction
values = [0, 0, 255, 255, 0]


def split_bytes(val):
	"""
	splits bytes for CAN msg
	reconstruct value with: high * 256 + low
	param val: 	int16
	"""
	high = val // 256	# MSB
	low = val % 256		# LSB
	return high, low


def calc_turn_radius(raw, s=0.45):
	"""
	param raw:	raw input from joystick
	param s:	front half length of rover
	"""
    	# turn angle
	theta = float(raw) / 360
    	if theta <= -90:
		theta = -90
	elif theta >= 90:
		theta = 90
	radians = math.radians(theta)
    	# turn radius
	try:
		radius = abs(s / math.tan(radians))
	except ZeroDivisionError:
		# no turn angle, infinite turn radius
		radius = 2**16 - 1
	# multiplied by 100 to return centimeters and reduce joystick sensitivity
	radius = radius * 100
	if radius > 2**16 - 1:
		radius = 2**16 - 1	
	return int(radius)


def callback(data):
	"""
	param data:	data from ROS joy topic	
	"""
	# can_handler.check_status(ID)  # TODO
	# rospy.loginfo(data) # debug
	forward = data.axes[4]
	reverse = data.axes[5]
	turning = data.axes[0]
	if forward == 0.0 and reverse == 0.0:
		# joy bug: starts at zero. Should be 1.
		forward = 1.0
		reverse = 1.0
	# convert raw data to uniform values wrt. linux character device values
	# ROS joy gives float values between -1 and 1, while raw character device values
	# should be either 16-bit, 8-bit or boolean. 
	values[0] = -int((forward - 1) * 255/2)  # 8 bit
	values[1] = -int((reverse - 1) * 255/2)  # 8 bit
	if turning == 0:
		# no turn, infinite turn radius
		values[2] = 255
		values[3] = 255
	else:
		# turn
		raw = -turning * 65535 / 2  # 16 bit
		turn_radius = calc_turn_radius(raw)
		tr_MSB, tr_LSB = split_bytes(turn_radius)
		values[2] = int(tr_MSB)
		values[3] = int(tr_LSB)
		if raw < 0:
			# left turn
			values[4] = 0
		elif raw > 0:
			# right turn
			values[4] = 1
	if values[0] != 0 and values[1] != 0:
		# failsafe: no forward and reverse at same time
		values[0] = 0
		values[1] = 0
	# send wheel commands to CAN bus
	can_handler.send_msg(ID, values)


def joy_subscriber():
	# initialize ROS node 'wheels'
	rospy.init_node('wheels')
	# subscribe to the ROS topic 'joy'
	rospy.Subscriber('joy', Joy, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	joy_subscriber()
