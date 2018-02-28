#!/usr/bin/env python

# ROS imports
import rospy
from sensor_msgs.msg import Joy

# Utils
import can
import math

# Configure socketcan (python-can)
bustype = 'socketcan_ctypes'
channel = 'can0'
can.rc['interface'] = bustype
can.rc['channel'] = channel

# CAN bus init
bus = can.interface.Bus(channel=channel, bustype=bustype)
# CAN msg preparation. Msg ID is 0x100 for wheel commands
msg = can.Message(arbitration_id=0x100, data=[], extended_id=False)

# speed fwd, speed reverse,
# turn radius MSB, turn radius LSB
values = [0, 0, 255, 255]


# split bytes to MSB and LSB
def split_bytes(val):
	"""
	splits bytes for CAN msg
	reconstruct value with: high * 256 + low
	"""
	high = val // 256	# MSB
	low = val % 256		# LSB
	return high, low


# send to CAN
def send_msg(values):
    	"""
    	param values: list of raw values
    	"""
	msg.data = values
	try:
		bus.send(msg)
	except can.CanError:
		print('CAN error:', can.CanError)


# calculate turn radius. Change s if needed
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
	# multiplied by 1000 to return millimeters and reduce joystick sensitivity
	return int(radius * 1000)


# handle subscription
def callback(data):
	# rospy.loginfo(data) # debug
	forward = data.axes[4]
	reverse = data.axes[5]
	turning = data.axes[0]
	if forward == 0.0 and reverse == 0.0:
		# bug: starts at zero. Should be 1.
		forward = 1.0
		reverse = 1.0
	# convert raw data to uniform values wrt. chosen standards
	values[0] = -int((forward - 1) * 255/2)
	values[1] = -int((reverse - 1) * 255/2)
	if turning == 0:
		# no turn
		values[2] = 255
		values[3] = 255
	else:
		# turn
		raw = turning * 65535 / 2
		turn_radius = calc_turn_radius(raw)
		tr_MSB, tr_LSB = split_bytes(turn_radius)
		values[2] = int(tr_MSB)
		values[3] = int(tr_LSB)
	if values[0] != 0 and values[1] != 0:
		# failsafe: no forward and reverse at same time
		values[0] = 0
		values[1] = 0
	send_msg(values)



# subscribe
def joy_subscriber():
	# initialize ROS node key_subscriber
	rospy.init_node('wheels')
	# subscribe to the ROS topic key_events
	rospy.Subscriber('joy', Joy, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	joy_subscriber()
