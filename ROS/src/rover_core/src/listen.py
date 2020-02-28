#!/usr/bin/python
"""This script is an example ros subscriber"""

import rospy
from std_msgs.msg import String

def callback(data):
    """Called when a message is received."""
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)

def listener():
    """Subscribes to a topic and sits here listening"""
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)


    rospy.spin()


if __name__ == '__main__':
    listener()
