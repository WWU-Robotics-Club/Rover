#!/usr/bin/python
"""This script is an example ros subscriber"""


import sys
import rospy
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from threading import Lock

'''
def callback(data):
    """Called when a message is received."""
    #rospy.loginfo(rospy.get_caller_id() + "\n I heard %s", data.pose)

    #left = np.array(data.data)
    #Left_D = data.D
    #height = data.height

    #print(Left_D)
    #print(height)
    #print(data.is_bigendian)
    print(data.header.stamp)
    

#def callback1(data):
    """Called when a message is received."""
    #rospy.loginfo(rospy.get_caller_id() + "\n I heard %s", data.pose)

    #left = data.D
    #print(left)


def getPose():
    """Subscribes to a topic and sits here listening"""
    rospy.init_node('getPose', anonymous=True)

    rospy.Subscriber("/camera/fisheye1/image_raw", Image, callback)
    #rospy.Subscriber("/camera/fisheye1/camera_info", CameraInfo, callback)
    #rospy.Subscriber("/camera/fisheye2/camera_info", CameraInfo, callback)


    rospy.spin()


if __name__ == '__main__':
    info = getPose()
'''


face_cascade = cv2.CascadeClassifier('/home/khalil/Documents/haarcascade_frontalface_default.xml') 

# https://github.com/Itseez/opencv/blob/master 
# /data/haarcascades/haarcascade_eye.xml 
# Trained XML file for detecting eyes 
eye_cascade = cv2.CascadeClassifier('/home/khalil/Documents/haarcascade_eye.xml') 

controller_cascade = cv2.CascadeClassifier('/home/khalil/Documents/cascade_controller.xml')





class image_converter:    
    def __init__(self):
        
        self.bridge = CvBridge()
        self.fisheye1()
        rospy.spin()

    def fisheye1(self):
        rospy.init_node('getImage', anonymous=True)

        self.image_sub = rospy.Subscriber("/camera/fisheye2/image_raw", Image, self.newWindow)

    def newWindow(self, data):

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)

        # reads frames from a camera 
        img = self.cv_image

        # convert to gray scale of each frames 
        gray = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

        # DETECT THE OBJECT USING THE CASCADE
        
         # Detects faces of different sizes in the input image 
        #faces = face_cascade.detectMultiScale(gray, 1.3, 5)
        controllers = controller_cascade.detectMultiScale(gray, 1.3, 5)
        # DISPLAY THE DETECTED OBJECTS
        for (x,y,w,h) in controllers: 
            cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,255), 3)
            cv2.putText(img, 'Controller', (x, y-5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,0,255), 2)
            roi_color = img[y:y+h, x:x+w]


        # Display an image in a window 
        cv2.imshow('img',img) 

        #cv2.waitKey(1)
        # Wait for Esc key to stop 
        k = cv2.waitKey(30) & 0xff
        
        #if k == 27: 
        
        
        
        
        
        #cv2.imshow('Image Window', self.cv_image)

        #cv2.waitKey(1)
        
        
        
        
        
        
        
        #disp_color = cv2.applyColorMap(cv_image,cv2.COLORMAP_JET) 

'''
def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
'''

if __name__ == '__main__':
    #main(sys.argv)
    ic = image_converter()