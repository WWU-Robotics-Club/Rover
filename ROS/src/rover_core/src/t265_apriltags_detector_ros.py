# Example code provided by https://github.com/swatbotics/apriltag
from __future__ import division
from __future__ import print_function

# Import AprilTag library
import apriltag

# Import OpenCV, numpy, & ROS libraries
import cv2
import rospy
import numpy
import collections
from math import tan, pi
from threading import Lock
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from argparse import ArgumentParser
from cv_bridge import CvBridge, CvBridgeError


class apriltag_detection:
    def __init__(self):
        self.image_sub = None
        self.right_sub = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.rotation = None

        # Set up mutex to share data threads
        self.frame_mutex = Lock()

        self.bridge = CvBridge()
        self.fisheye2_data()
        rospy.spin()

    # Retrieve data from T265 right fisheye camera through its topics
    def fisheye2_data(self):
        rospy.init_node('AprilTag', anonymous=True)

        self.image_sub = rospy.Subscriber("/camera/fisheye2/image_raw", Image, self.right_fisheye)
        self.right_sub = rospy.Subscriber("/camera/fisheye2/camera_info", CameraInfo, self.CameraParams)
        self.pub = rospy.Publisher('/apriltag/pos', Pose, queue_size=10)

    # Get Camera Parameters & display Tags
    def CameraParams(self, data):
        self.fx = data.K[0]
        self.fy = data.K[4]
        self.cx = data.K[2]
        self.cy = data.K[5]

        self.disp_AprilTag()

    # Convert right fisheye image through CV Bridge to provide live video
    def RightImage(self, data):
        self.frame_mutex.acquire()
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)

        self.frame_mutex.release()

    # Draws box around AprilTags with Pose Lines
    def _draw_pose(self, overlay, camera_params, tag_size, pose, z_sign=1):

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

        rvec, _ = cv2.Rodrigues(pose[:3, :3])
        tvec = pose[:3, 3]

        # Rotation Vector
        self.rotation = rvec

        dcoeffs = numpy.zeros(5)

        ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

        ipoints = numpy.round(ipoints).astype(int)

        ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

        for i, j in edges:
            cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)

    # Function to display AprilTags
    def disp_AprilTag(self):

        parser = ArgumentParser(
            description='AprilTag Detector')

        apriltag.add_arguments(parser)

        options = parser.parse_args()

        window = 'Camera'
        cv2.namedWindow(window)

        # AprilTag Detector Function
        detector = apriltag.Detector(options,
                                     searchpath=apriltag._get_demo_searchpath())
        if True:

            frame = self.cv_image

            camera_params = [self.fx, self.fy, self.cx, self.cy]
            # Detects the tags
            detections, dimg = detector.detect(frame, return_image=True)
            overlay = frame // 2 + dimg[:, :] // 2

            num_detections = len(detections)
            print('Detected {} tags.\n'.format(num_detections))

            # AprilTag ID associated with Charging Station
            ChargingStation = 33

            # Loop to find tags and print how many there are
            for i, detection in enumerate(detections):
                print('Detection {} of {}:'.format(i+1, num_detections))
                print()

                # Retreive Pose information
                pose, e0, e1 = detector.detection_pose(detection,
                                                       camera_params,
                                                       1)

                # Draws box around the AprilTag
                self._draw_pose(overlay,
                                camera_params,
                                1,
                                pose)

                # Looks for the Charging AprilTag and provide the position
                if detection.tag_id == ChargingStation:
                    print(detection.tag_id, 'Charging Station Detected')

                    # Rotation about x
                    roll = self.rotation[2]
                    # Rotation about y
                    pitch = self.rotation[0]
                    # Rotation about z
                    yaw = self.rotation[1]

                    # Publish positon and orientation to topic
                    p = Pose()
                    p.position.x = pose[0][3]
                    p.position.y = pose[1][3]
                    p.position.z = pose[2][3]

                    p.orientation.x = roll
                    p.orientation.y = pitch
                    p.orientation.z = yaw
                    p.orientation.w = 0.0
                    self.pub.publish(p)

                print()

                # Print Pose and errors
                # Last column provides x, y, z coordinates
                print(detection.tostring(
                    collections.OrderedDict([('Pose', pose),
                                             ('InitError', e0),
                                             ('FinalError', e1)]),
                    indent=2))

                print()

            # Display camera stream of AprilTag detection
            cv2.imshow(window, overlay)

            k = cv2.waitKey(1) & 0xFF


if __name__ == '__main__':
    ad = apriltag_detection()
