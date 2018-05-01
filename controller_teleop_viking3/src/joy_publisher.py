#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import click
from comms import udpserver


def joy_publisher():
    # initialize ROS node joy_publisher
    rospy.init_node('joy_publisher')
    # publish joypad controller events to the ROS topic joy_events
    pub = rospy.Publisher('joy_events', String, queue_size=10)
    # 100 Hz
    rate = rospy.Rate(100)
    # publish
    while not rospy.is_shutdown():
        data = udpserver.server()
        pub.publish(data)
        rate.sleep()


if __name__ == '__main__':
    try:
        joy_publisher()
    except rospy.ROSInterruptException:
        pass
