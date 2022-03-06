#!/usr/bin/env python3

# Copyright 2021 Memristor Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2
from cv2 import aruco
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster
"""
This class receives video and camera parameters from the central RasPi Cam,
receives the static transform from map to camera in order to detect wrong orientations
and sends TF of Aruco tags.
"""


class DetectedRobots(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.image_subscription = self.create_subscription(
            Image, '/cam/cam_central/RasPi0', self.image_listener_callback, 10)
        self.image_subscription
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/cam/cam_central/RasPi0/camera_info',
            self.camera_info_listener_callback, 10)
        self.camera_info_subscription
        self._tf_publisher = TransformBroadcaster(self)

        self.br = CvBridge()
        self.dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.params = aruco.DetectorParameters_create()
        # Side length of the ArUco marker in meters
        self.aruco_robot_length = 0.07
        self.aruco_42_length = 0.1
        self.camera_translation = [-0.141, 1.417, 1.184]
        self.camera_rotation = R.from_quat([-0.005, 0.962, -0.272, 0.008])

    def image_listener_callback(self, data):
        # self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        rvecs, tvecs, ids = self.get_aruco_pose(current_frame)
        self.send_pose_tf2(rvecs, tvecs, ids)

    def camera_info_listener_callback(self, data):
        # self.get_logger().info('Receiving camera info frame')
        # self.cameraMatrix = np.array([[570.34, 0, 1920 / 2],
        #                               [0, 570.34, 1080 / 2], [0, 0, 1]])
        # self.distCoeffs = np.zeros((5, 1))
        self.cameraMatrix = np.array([[data.k[0], data.k[1], data.k[2]],
                                      [data.k[3], data.k[4], data.k[5]],
                                      [data.k[6], data.k[7], data.k[8]]])
        self.distCoeffs = np.array([[i] for i in data.d])

    def send_pose_tf2(self, rvecs, tvecs, ids):
        if ids is not None:
            for i in range(len(rvecs)):
                if ids[i] < 10 or ids[i] == 42:
                    translation = tvecs[i, 0]
                    # https://answers.opencv.org/question/161369/retrieve-yaw-pitch-roll-from-rvec/
                    # https://stackoverflow.com/questions/12933284/rodrigues-into-eulerangles-and-vice-versa
                    # https://github.com/opencv/opencv/issues/8813
                    rmat = cv2.Rodrigues(rvecs[i])[0]
                    r = R.from_matrix(rmat)
                    rotation = r.as_quat()

                    if ids[i] == 42:
                        if self.check_alignment(
                                r, [0, 1, 0]) and self.check_alignment(
                                    r, [0, 0, 1]):
                            self.make_transforms('camera', f'marker_{ids[i]}',
                                                 translation, rotation)
                    else:
                        if self.check_alignment(r, [0, 0, 1]):
                            self.make_transforms('camera', f'marker_{ids[i]}',
                                                 translation, rotation)

    def check_alignment(self, r, axis):
        # It would probably be better to say >1/sqrt(2) instead of >0:
        dot_product = np.dot(r.apply(axis),
                             self.camera_rotation.inv().apply(axis))
        # print(dot_product)
        if dot_product > 0.707:
            return True
        return False

    def make_transforms(self, frame_id, child_frame_id, translation, rotation):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = frame_id
        static_transformStamped.child_frame_id = child_frame_id
        static_transformStamped.transform.translation.x = translation[0]
        static_transformStamped.transform.translation.y = translation[1]
        static_transformStamped.transform.translation.z = translation[2]
        static_transformStamped.transform.rotation.x = rotation[0]
        static_transformStamped.transform.rotation.y = rotation[1]
        static_transformStamped.transform.rotation.z = rotation[2]
        static_transformStamped.transform.rotation.w = rotation[3]
        self._tf_publisher.sendTransform(static_transformStamped)

    def get_aruco_pose(self, frame):
        (corners, ids,
         rejected) = aruco.detectMarkers(frame,
                                         self.dict,
                                         parameters=self.params,
                                         cameraMatrix=self.cameraMatrix,
                                         distCoeff=self.distCoeffs)
        tvecs = rvecs = None
        if ids is not None:
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.aruco_robot_length, self.cameraMatrix,
                self.distCoeffs)
            rvecs_42, tvecs_42, obj_points_42 = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.aruco_42_length, self.cameraMatrix,
                self.distCoeffs)

            for i in range(len(rvecs)):
                if ids[i] == 42:
                    tvecs[i] = tvecs_42[i]
                    rvecs[i] = rvecs_42[i]

            self.show_aruco_pose(frame, corners, rvecs, tvecs, ids)

        return rvecs, tvecs, ids

    def show_aruco_pose(self, frame, corners, rvecs, tvecs, ids):
        aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(rvecs)):
            if ids[i] <= 10:
                aruco.drawAxis(frame, self.cameraMatrix, self.distCoeffs,
                               rvecs[i], tvecs[i], self.aruco_robot_length)
            elif ids[i] == 42:
                aruco.drawAxis(frame, self.cameraMatrix, self.distCoeffs,
                               rvecs[i], tvecs[i], self.aruco_42_length)
        cv2.imshow('camera', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    detected_robots = DetectedRobots()
    rclpy.spin(detected_robots)
    detected_robots.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
