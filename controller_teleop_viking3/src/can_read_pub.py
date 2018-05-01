#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import threading, json
from comms import can_handler

# CAN reads (ID: data)
statuses = {}
mutex = threading.Lock()


def can_read_pub():
	global statuses
	# initialize ROS node can_read_publisher
	rospy.init_node('can_read_publisher')
	# publish joypad controller events to the ROS topic CAN_statuses
	pub = rospy.Publisher('CAN_statuses', String, queue_size=10)
	# 100Hz
	rate = rospy.Rate(100)
	# publish
	while not rospy.is_shutdown():
		can_handler.check_status(mutex, statuses)
		statuses = dict(statuses)
		for k in statuses:
			statuses[k] = list(statuses[k])
		pub.publish(json.dumps(statuses))
		rate.sleep()


if __name__ == '__main__':
    try:
        can_read_pub()
    except rospy.ROSInterruptException:
        pass
