#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import json
from comms import can_handler


# msg ID is 0x100 for wheel commands
ID = 0x100

# speed MSB, speed LSB,
# turn radius B1, turn radius B2, turn radius B3, turn radius B4
values = [0, 0, 255, 255, 255, 255]


def callback(data):
	# rospy.loginfo(rospy.get_caller_id() + " heard %s", data.data) # debug
	data = json.loads(data.data)
	# print('Controller axes:', data['Axes']) 	# debug
	# print('Controller buttons:', data['Buttons'])	# debug
	turning = data['Axes']['0']
	speed = data['Axes']['1']
	# speed:
	s_MSB, s_LSB = can_handler.split_bytes(speed, 2)
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
		# using 32-bit value for turning radius
		if turning < 0:
			turning = turning * (2**16)
		else:
			turning = (turning + 1) * (2**16) - 1
		tr_B1, tr_B2, tr_B3, tr_B4 = can_handler.split_bytes(turning, 4)
		values[2] = int(tr_B1)
		values[3] = int(tr_B2)
		values[4] = int(tr_B4)
		values[5] = int(tr_B4)
	# send wheel commands to CAN bus
	can_handler.send_msg(ID, values)


def wheel_control():
	# initialize ROS node 'wheels'
	rospy.init_node('wheels')
	# subscribe to the ROS topic 'joy_events'
	rospy.Subscriber('joy_events', String, callback)
	# keep python from exiting until this node is stopped
	rospy.spin()


if __name__ == "__main__":
	wheel_control()
