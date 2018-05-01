#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import json
from comms import can_handler

# msg ID is 0x300 for digger commands
ID = 0x300
# dig normal/reverse, digger speed
values = [0, 0]
old_vals = [0, 0]


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	global ID
	global values
	global old_vals
	# fetch data from joy_events
	data = json.loads(data.data)
	# fetch joypad controller input and update values
	direction = data['Buttons']['4']	# normal/reverse
	digging = data['Axes']['2']			# digging speed
	values[0] = direction
	values[1] = digging
	# send digger commands to CAN bus
	if values != old_vals:
		old_vals = values[:]
		can_handler.send_msg(ID, values)


def digger_control():
	# initialize ROS node 'digger'
	rospy.init_node('digger')
	# subscribe to the ROS topic 'joy_events'
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	digger_control()
