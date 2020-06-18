# Example code provided by https://github.com/swatbotics/apriltag
from __future__ import division
from __future__ import print_function

import cv2
import rospy
import numpy
import apriltag
import collections
from math import tan, pi
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from argparse import ArgumentParser
from cv_bridge import CvBridge, CvBridgeError

def _draw_pose(overlay, camera_params, tag_size, pose, z_sign=1):

    opoints = numpy.array([
        -1, -1, 0,
         1, -1, 0,
         1,  1, 0,
        -1,  1, 0,
        -1, -1, -2*z_sign,
         1, -1, -2*z_sign,
         1,  1, -2*z_sign,
        -1,  1, -2*z_sign,
    ]).reshape(-1, 1, 3) * 0.5*tag_size

    edges = numpy.array([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6,
        6, 7,
        7, 4
    ]).reshape(-1, 2)
        
    fx, fy, cx, cy = camera_params

    K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3,:3])
    tvec = pose[:3, 3]

    dcoeffs = numpy.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = numpy.round(ipoints).astype(int)
    
    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)


class image_converter:
    def __init__(self):

        self.bridge = CvBridge()
        self.fisheye1()
        rospy.spin()

    def fisheye1(self):
        rospy.init_node('AprilTag', anonymous=True)
        
        self.image_sub = rospy.Subscriber("/camera/fisheye1/image_raw", Image, self.newWindow)
        self.pub = rospy.Publisher('/apriltag/pos', Pose, queue_size=10)


    def newWindow(self, data):

        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)

        parser = ArgumentParser(
            description='test apriltag Python bindings')

      
        apriltag.add_arguments(parser)

        options = parser.parse_args()
        

        window = 'Camera'
        cv2.namedWindow(window)
        
        
        detector = apriltag.Detector(options,
                                    searchpath=apriltag._get_demo_searchpath())
        if True:

            frame = self.cv_image
            
            min_disp = 0
            num_disp = 112 - min_disp
            max_disp = min_disp + num_disp
            stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
            stereo_height_px = 300         # 300x300 pixel stereo output
            stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)
            stereo_cx = (stereo_height_px - 1)/2 + max_disp
            stereo_cy = (stereo_height_px - 1)/2
            camera_params = [stereo_focal_px, stereo_focal_px, stereo_cx, stereo_cy]
            
            detections, dimg = detector.detect(frame, return_image=True)
            

            overlay = frame // 2 + dimg[:, :] // 2
            
            num_detections = len(detections)
            print('Detected {} tags.\n'.format(num_detections))

            ChargingStation = 33

            for i, detection in enumerate(detections):
                print('Detection {} of {}:'.format(i+1, num_detections))
                print()
                if detection.tag_id == ChargingStation:
                    print(detection.tag_id, 'Charging Station Detected')
                print()

                pose, e0, e1 = detector.detection_pose(detection,
                                                    camera_params,
                                                    1)
                 # Draws box around the AprilTag               
                _draw_pose(overlay,
                        camera_params,
                        1,
                        pose)
              
                
                print ('X:', pose[0][3], '\nY:', pose[1][3], '\nZ:', pose[2][3])

                print(detection.tostring(
                    collections.OrderedDict([('Pose',pose),
                                            ('InitError', e0),
                                            ('FinalError', e1)]),
                    indent=2))
                    
                print()
            
        
            cv2.imshow(window, overlay)

            k = cv2.waitKey(1) & 0xFF

if __name__ == '__main__':
    image_converter()