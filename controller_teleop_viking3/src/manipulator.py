#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import json
from comms import can_handler

# msg ID is 0x200 for manipulator commands
ID = 0x200
# link select, rise MSB, rise LSB, sway MSB, sway LSB
values = [0, 0, 0, 0, 0]
old_vals = [0, 0, 0, 0, 0]


def callback(data):
	"""
	param data:	data from ROS joy topic
	"""
	global ID
	global values
	global old_vals

	# fetch data from joy_events
	data = json.loads(data.data)
	# fetch joypad controller input
	link_select = data['Buttons']['7']
	rise = data['Axes']['3']
	sway = data['Axes']['4']
	r1, r2 = can_handler.split_bytes(rise, 2)
	s1, s2 = can_handler.split_bytes(sway, 2)
	values = [link_select, r1, r2, s1, s2]
	# send manipulator commands to CAN bus
	if values != old_vals:
		old_vals = values[:]
		can_handler.send_msg(ID, values)


def manipulator_control():
	# initialize ROS node 'manipulator'
	rospy.init_node('manipulator')
	# subscribe to the ROS topic 'joy_events'
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	manipulator_control()
