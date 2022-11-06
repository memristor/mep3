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
from tf2_ros.buffer import Buffer


class ArucoDetector(Node):
    """
    Detects Aruco tags from the central tracking device.

    This class receives video and camera parameters from the central RasPi Cam,
    receives the static transform from map to camera in order to detect wrong
    orientations and sends TF of Aruco tags.
    """

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
        self.aruco_robot_tag_length = 0.07
        self.aruco_table_tag_length = 0.1

    def image_listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        rvecs, tvecs, ids = self.get_aruco_pose(current_frame)
        self.send_pose_tf2(rvecs, tvecs, ids)

    def camera_info_listener_callback(self, data):
        self.cameraMatrix = np.array([[data.k[0], data.k[1], data.k[2]],
                                      [data.k[3], data.k[4], data.k[5]],
                                      [data.k[6], data.k[7], data.k[8]]])
        self.distCoeffs = np.array([[i] for i in data.d])

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
                corners, self.aruco_robot_tag_length, self.cameraMatrix,
                self.distCoeffs)
            rvecs_table, tvecs_table, obj_points_table = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.aruco_table_tag_length, self.cameraMatrix,
                self.distCoeffs)

            for i in range(len(rvecs)):
                # Table markers are TAG_20, TAG_21, TAG_22 and TAG_23
                if ids[i] >= 20 and ids[i] <= 23:
                    tvecs[i] = tvecs_table[i]
                    rvecs[i] = rvecs_table[i]

            self.show_aruco_pose(frame, corners, rvecs, tvecs, ids)

        return rvecs, tvecs, ids

    def send_pose_tf2(self, rvecs, tvecs, ids):
        if ids is not None:
            for i in range(len(rvecs)):
                translation = tvecs[i, 0]
                # https://answers.opencv.org/question/161369/retrieve-yaw-pitch-roll-from-rvec/
                # https://stackoverflow.com/questions/12933284/rodrigues-into-eulerangles-and-vice-versa
                # https://github.com/opencv/opencv/issues/8813
                rmat = cv2.Rodrigues(rvecs[i])[0]
                r = R.from_matrix(rmat)
                rotation = r.as_quat()

                self.make_transforms('camera', f'raw_marker_{ids[i]}',
                                     translation, rotation)

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

    def show_aruco_pose(self, frame, corners, rvecs, tvecs, ids):
        aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(rvecs)):
            if ids[i] <= 10:
                aruco.drawAxis(frame, self.cameraMatrix, self.distCoeffs,
                               rvecs[i], tvecs[i], self.aruco_robot_tag_length)
            elif ids[i] == 42:
                aruco.drawAxis(frame, self.cameraMatrix, self.distCoeffs,
                               rvecs[i], tvecs[i], self.aruco_table_tag_length)
        cv2.imshow('camera', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
