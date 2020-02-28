#!/usr/bin/python
"""This script is an example ros publisher"""

import rospy
from std_msgs.msg import String

def talker():
    """Runs a loop publishing a message"""
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        test_str = "Hello world %s" % rospy.get_time()
        rospy.loginfo(test_str)
        pub.publish(test_str)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


#val = input ("Enter your value: ")
#print(val)
