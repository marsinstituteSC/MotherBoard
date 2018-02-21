#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import click
from udp import udpserver


def joy_publisher():
    # initialize ROS node joy_publisher
    rospy.init_node('joy_publisher') # , anonymous=True
    # publish key events to the ROS topic joy_events
    pub = rospy.Publisher('joy_events', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        data = udpserver.server()
        pub.publish(data)
        rate.sleep()


if __name__ == "__main__":
    try:
        joy_publisher()
    except rospy.ROSInterruptException:
        pass
