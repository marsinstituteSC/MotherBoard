#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import json
from comms import can_handler

# msg ID is 0x250 for drill commands
ID = 0x250
# clockwise/counter-clockwise, drill speed
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
	drilling = data['Axes']['5']		# drilling speed
	direction = data['Buttons']['5']	# normal/reverse
	values[0] = direction
	values[1] = drilling
	# send drill commands to CAN bus
	if values != old_vals:
		old_vals = values[:]
		can_handler.send_msg(ID, values)


def drill_control():
	# initialize ROS node 'drill'
	rospy.init_node('drill')
	# subscribe to the ROS topic 'joy_events'
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	drill_control()
