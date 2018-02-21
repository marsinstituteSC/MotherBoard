#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import click
from tcp import tcpserver


def key_publisher():
    # initialize ROS node key_publisher
    rospy.init_node('joy_publisher') # , anonymous=True
    # publish key events to the ROS topic key_events
    pub = rospy.Publisher('joy_events', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        data = tcpserver.server()
        pub.publish(data)
        rate.sleep()


if __name__ == "__main__":
    try:
        key_publisher()
    except rospy.ROSInterruptException:
        pass
