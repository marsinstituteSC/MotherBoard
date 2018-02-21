#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import can

# Nodes
from nodes import wheel_turner, wheel_motor

# Configure socketcan for python 2.7 (python-can)
bustype = 'socketcan_ctypes'
channel = 'can0'
can.rc['interface'] = bustype
can.rc['channel'] = channel

# CAN bus and message preparation
bus = can.interface.Bus(channel=channel, bustype=bustype)
msg = can.Message(arbitration_id=0x200, data=[], extended_id=False)

# 0x7F = 0 for speed and angle.
values = [0x7F, 0x7F, 0x00] # speed, turn angle, turn radius

# helper function for sending CAN messages
def sendMsg(values):
	msg.data = values
	try:
		bus.send(msg)
	except can.CanError:
		print("Message NOT sent! Error:", can.CanError)

# handle subscription
def callback(data):
	# rospy.loginfo(rospy.get_caller_id() + " heard %s", data.data) # debug
	if data.data == ' ':
		# reset and stop if stop button (space) is pressed
		values[0] = 0x7F
		values[1] = 0x7F
		values[2] = 0x00
		sendMsg(values)
	elif data.data == 'w':
		# go forward
		values[0] = wheel_motor.drive('forward', values[0])
		sendMsg(values)
	elif data.data == 'a':
		# go left
		values[1] = wheel_turner.turn('left', values[1])
		sendMsg(values)
	elif data.data == 'd':
		# go right
		values[1] = wheel_turner.turn('right', values[1])
		sendMsg(values)
	elif data.data == 's':
		# reverse
		values[0] = wheel_motor.drive('reverse', values[0])
		sendMsg(values)

# subscribe
def key_subscriber():
	# initialize ROS node key_subscriber
	rospy.init_node('key_subscriber')
	# subscribe to the ROS topic key_events
	rospy.Subscriber('key_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	key_subscriber()
