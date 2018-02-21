#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import can
import json

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

values = [] # speed, turn angle, turn radius

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
	data = json.loads(data.data)
	print('Type of data:', type(data))
	print('Controller axes:', data['Axes'])
	print('Controller buttons:', data['Buttons'])

# subscribe
def joy_subscriber():
	# initialize ROS node joy_subscriber
	rospy.init_node('joy_subscriber')
	# subscribe to the ROS topic joy_events
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	joy_subscriber()
