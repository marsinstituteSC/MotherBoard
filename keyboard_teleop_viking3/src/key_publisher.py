#!/usr/bin/env python

# ROS imports
import rospy
from std_msgs.msg import String

# Utils
import click


def key_publisher():
    # initialize ROS node key_publisher
    rospy.init_node('key_publisher') # , anonymous=True
    # publish key events to the ROS topic key_events
    pub = rospy.Publisher('key_events', String, queue_size=10)

    while not rospy.is_shutdown():
        key = click.getchar()
        pub.publish(key)
        # rospy.loginfo(key) # debug
        
    # TODO: add check if stop key is pressed?


if __name__ == "__main__":
    try:
        key_publisher()
    except rospy.ROSInterruptException:
        pass
