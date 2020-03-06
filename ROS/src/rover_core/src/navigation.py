#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist



def callback(data):
    rospy.loginfo(rospy.get_caller_id, data.data)

def velocity_pub():
    pub = ros.Publisher('wheel_velocity', String, queue_size=10)
    rospy.init_node('navigation', anonymous=True)
    rate = rospy.Rate(10)

def position_sub():
    rospy.init_node('navigation', anonymous=True)
    rospy.Subscriber("wheel_position", String, callback)

    rospy.spin()

''' def camerapos_sub(_):      
    rospy.init_node('navigation', anonymous=True)
    rospy.Subscriber("camera_position", String, callback)

    rospy.spin() '''

def web_sub():
    rospy.init_node('navigation', anonymous=True)
    rospy.Subscriber("/web_vel", Twist, callback)

if __name__ == '__main__':
    pos1 = 5
    pos2 = 10
    pos3 = 10
    pos4 = 5

    message = str(pos1) + "," + str(pos2) + "," + str(pos3) + "," + str(pos4)
    
    print(message)