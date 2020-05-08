#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# Python 2/3 compatibility
from __future__ import print_function

"""
This example shows how to use T265 intrinsics and extrinsics in OpenCV to
asynchronously compute depth maps from T265 fisheye images on the host.

T265 is not a depth camera and the quality of passive-only depth options will
always be limited compared to (e.g.) the D4XX series cameras. However, T265 does
have two global shutter cameras in a stereo configuration, and in this example
we show how to set up OpenCV to undistort the images and compute stereo depth
from them.

Getting started with python3, OpenCV and T265 on Ubuntu 16.04:

First, set up the virtual enviroment:

$ apt-get install python3-venv  # install python3 built in venv support
$ python3 -m venv py3librs      # create a virtual environment in pylibrs
$ source py3librs/bin/activate  # activate the venv, do this from every terminal
$ pip install opencv-python     # install opencv 4.1 in the venv
$ pip install pyrealsense2      # install librealsense python bindings

Then, for every new terminal:

$ source py3librs/bin/activate  # Activate the virtual environment
$ python3 t265_stereo.py        # Run the example
"""


# Import OpenCV and numpy
import cv2
import rospy
import numpy as np
from math import tan, pi
from threading import Lock
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Imu, Image, CameraInfo

"""
In this section, we will set up the functions that will translate the camera
intrinsics and extrinsics from librealsense into parameters that can be used
with OpenCV.

The T265 uses very wide angle lenses, so the distortion is modeled using a four
parameter distortion model known as Kanalla-Brandt. OpenCV supports this
distortion model in their "fisheye" module, more details can be found here:

https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
"""
class StereoCamera:
    def __init__(self):
        self.left_sub = None
        self.right_sub = None
        self.Left_K = None
        self.Right_K = None
        self.Left_D = None
        self.Right_D = None
        self.height = None
        self.width = None
        self.R_cam = None
        
        # Set up a mutex to share data between threads 
        self.frame_mutex = Lock()
        self.frame_data = {"left"  : None,
                    "right" : None,
                    "timestamp_ms" : None
                    }

        self.bridge = CvBridge()
        self.getCam_Info()
        rospy.spin()        

    def getCam_Info(self):
        rospy.init_node('getCam_Info', anonymous=True)
        
        self.left_sub = rospy.Subscriber("/camera/fisheye1/camera_info", CameraInfo, self.camera1)
        self.right_sub = rospy.Subscriber("/camera/fisheye2/camera_info", CameraInfo, self.camera2)
        
        self.left_img = rospy.Subscriber("/camera/fisheye1/image_raw", Image, self.LeftImage)
        self.right_img = rospy.Subscriber("/camera/fisheye2/image_raw", Image, self.RightImage )

        

    def camera1(self, data):
        self.Left_K = np.resize(data.K, (3,3))
        self.Left_D = np.resize(data.D, (1,4))
        self.height = data.height
        self.width = data.width
        self.R_cam = np.resize(data.R, (3,3))

        self.compute()

    def camera2(self, data):
        self.Right_K = np.resize(data.K, (3,3))
        self.Right_D = np.resize(data.D, (1,4))

    def LeftImage(self, data):
        self.frame_mutex.acquire()
        try:
            self.frame_data["left"] = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.frame_data["timestamp_ms"] = data.header.stamp
        except CvBridgeError as e:
            print(e)

        self.frame_mutex.release()

    def RightImage(self, data):
        self.frame_mutex.acquire()
        try:
            self.frame_data["right"] = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self.frame_data["timestamp_ms"] = data.header.stamp
        except CvBridgeError as e:
            print(e)

        self.frame_mutex.release()


    """
    This callback is called on a separate thread, so we must use a mutex
    to ensure that data is synchronized properly. We should also be
    careful not to do much work on this thread to avoid data backing up in the
    callback queue.
    """
    '''
    def callback(frame):
        global frame_data
        if frame.is_frameset():
            frameset = frame.as_frameset()
            f1 = frameset.get_fisheye_frame(1).as_video_frame()
            f2 = frameset.get_fisheye_frame(2).as_video_frame()
            left_data = np.asanyarray(f1.get_data())
            right_data = np.asanyarray(f2.get_data())
            ts = frameset.get_timestamp()
            frame_mutex.acquire()
            frame_data["left"] = left_data
            frame_data["right"] = right_data
            frame_data["timestamp_ms"] = ts
            frame_mutex.release()
    '''

    def compute(self):
        try:
            # Set up an OpenCV window to visualize the results
            WINDOW_TITLE = 'Realsense'
            cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

            # Configure the OpenCV stereo algorithm. See
            # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
            # description of the parameters
            window_size = 5
            min_disp = 0
            # must be divisible by 16
            num_disp = 112 - min_disp
            max_disp = min_disp + num_disp
            stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                        numDisparities = num_disp,
                                        blockSize = 16,
                                        P1 = 8*3*window_size**2,
                                        P2 = 32*3*window_size**2,
                                        disp12MaxDiff = 1,
                                        uniquenessRatio = 10,
                                        speckleWindowSize = 100,
                                        speckleRange = 32)


        
            # Get the relative extrinsics between the left and right camera

            R = np.array([[0.999986708, .000505891512, .00513231847],
                        [-0.000483156764, 0.999990046, 0.00442992477],
                        [0.00513450988, -0.00442738598, 0.999977052]])

            T = np.array([-0.0645980090, 0.00000026935603, -0.0000956418808])

            # We need to determine what focal length our undistorted images should have
            # in order to set up the camera matrices for initUndistortRectifyMap.  We
            # could use stereoRectify, but here we show how to derive these projection
            # matrices from the calibration and a desired height and field of view

            # We calculate the undistorted focal length:
            #
            #         h
            # -----------------
            #  \      |      /
            #    \    | f  /
            #     \   |   /
            #      \ fov /
            #        \|/
            stereo_fov_rad = 90 * (pi/180)  # 90 degree desired fov
            stereo_height_px = 300          # 300x300 pixel stereo output
            stereo_focal_px = stereo_height_px/2 / tan(stereo_fov_rad/2)

            # We set the left rotation to identity and the right rotation
            # the rotation between the cameras
            R_left = np.eye(3)
            R_right = R

            # The stereo algorithm needs max_disp extra pixels in order to produce valid
            # disparity on the desired output region. This changes the width, but the
            # center of projection should be on the center of the cropped image
            stereo_width_px = stereo_height_px + max_disp
            stereo_size = (stereo_width_px, stereo_height_px)
            stereo_cx = (stereo_height_px - 1)/2 + max_disp
            stereo_cy = (stereo_height_px - 1)/2

            # Construct the left and right projection matrices, the only difference is
            # that the right projection matrix should have a shift along the x axis of
            # baseline*focal_length
            P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                            [0, stereo_focal_px, stereo_cy, 0],
                            [0,               0,         1, 0]])
            P_right = P_left.copy()
            P_right[0][3] = T[0]*stereo_focal_px

            # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
            # since we will crop the disparity later
            Q = np.array([[1, 0,       0, -(stereo_cx - max_disp)],
                        [0, 1,       0, -stereo_cy],
                        [0, 0,       0, stereo_focal_px],
                        [0, 0, -1/T[0], 0]])

            # Create an undistortion map for the left and right camera which applies the
            # rectification and undoes the camera distortion. This only has to be done
            # once
            m1type = cv2.CV_32FC1
            (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(self.Left_K, self.Left_D, R_left, P_left, stereo_size, m1type)
            (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(self.Right_K, self.Right_D, R_right, P_right, stereo_size, m1type)
            
            undistort_rectify = {"left"  : (lm1, lm2),
                                "right" : (rm1, rm2)}

            mode = "stack"
            if True:
                # Check if the camera has acquired any frames
                self.frame_mutex.acquire()
                valid = self.frame_data["timestamp_ms"] is not None
                self.frame_mutex.release()

                # If frames are ready to process
                if valid:
                    # Hold the mutex only long enough to copy the stereo frames
                    self.frame_mutex.acquire()
                    frame_copy = {"left"  : self.frame_data["left"].copy(),
                                "right" : self.frame_data["right"].copy()}
                    self.frame_mutex.release()

                    # Undistort and crop the center of the frames
                    center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                                map1 = undistort_rectify["left"][0],
                                                map2 = undistort_rectify["left"][1],
                                                interpolation = cv2.INTER_LINEAR),
                                        "right" : cv2.remap(src = frame_copy["right"],
                                                map1 = undistort_rectify["right"][0],
                                                map2 = undistort_rectify["right"][1],
                                                interpolation = cv2.INTER_LINEAR)}

                    # compute the disparity on the center of the frames and convert it to a pixel disparity (divide by DISP_SCALE=16)
                    disparity = stereo.compute(center_undistorted["left"], center_undistorted["right"]).astype(np.float32) / 16.0

                    # re-crop just the valid part of the disparity
                    disparity = disparity[:,max_disp:]

                    # convert disparity to 0-255 and color it
                    disp_vis = 255*(disparity - min_disp)/ num_disp
                    disp_color = cv2.applyColorMap(cv2.convertScaleAbs(disp_vis,1), cv2.COLORMAP_JET)
                    color_image = cv2.cvtColor(center_undistorted["left"][:,max_disp:], cv2.COLOR_GRAY2RGB)

                    if mode == "stack":
                        cv2.imshow(WINDOW_TITLE, np.hstack((color_image, disp_color)))
                    if mode == "overlay":
                        ind = disparity >= min_disp
                        color_image[ind, 0] = disp_color[ind, 0]
                        color_image[ind, 1] = disp_color[ind, 1]
                        color_image[ind, 2] = disp_color[ind, 2]
                        cv2.imshow(WINDOW_TITLE, color_image)
                key = cv2.waitKey(1)
                if key == ord('s'): mode = "stack"
                if key == ord('o'): mode = "overlay"
                if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                    pass
                

        finally:
            pass

if __name__ == '__main__':
    cam = StereoCamera()