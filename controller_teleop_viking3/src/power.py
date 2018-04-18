#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import json
from comms import can_handler

# msg ID is 0x050 for power commands
ID = 0x050
# power values
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
	# fetch joypad controller input
	option = data['Buttons']['6']
	green = data['Buttons']['0']
	red = data['Buttons']['1']
	if option == 1:
		if green == 1:
			values = [0xFF] 	# full power
		elif red == 1:
			values = [0x00] 	# sleep mode
		# send commands to CAN bus
		if values != old_vals:
			old_vals = values
			can_handler.send_msg(ID, values)


def power_control():
	# initialize ROS node 'power'
	rospy.init_node('power')
	# subscribe to the ROS topic 'joy_events'
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':
	power_control()
