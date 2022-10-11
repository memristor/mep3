#!/usr/bin/env python3

# Copyright 2022 Memristor Robotics
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
from tf2_ros.transform_listener import TransformListener


class ArucoDetector(Node):
    """
    Detects positions & orientations of Aruco markers from a camera frame.

    This class receives video and camera parameters from the central RasPi Cam,
    receives the static transform from map to camera in order to detect wrong
    orientations and sends TF of Aruco tags.
    """

    def __init__(self):
        super().__init__('aruco_detector')

        self.image_subscription = self.create_subscription(
            Image, '/cam/cam_central/RasPi0', self.image_listener_callback, 10)
        self.image_subscription

        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/cam/cam_central/RasPi0/camera_info',
            self.camera_info_listener_callback, 10)
        self.camera_info_subscription

        self._tf_publisher = TransformBroadcaster(self)
        self.static_tf_listener('map', 'camera_static')
        self.br = CvBridge()

        self.dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.params = aruco.DetectorParameters_create()

        # Side length of the ArUco marker in meters
        self.aruco_robot_length = 0.07
        self.aruco_42_length = 0.1

    def image_listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        rvecs, tvecs, ids = self.get_aruco_pose(current_frame)
        self.send_pose_tf2(rvecs, tvecs, ids)

    def camera_info_listener_callback(self, data):
        self.cameraMatrix = np.array([[data.k[0], data.k[1], data.k[2]],
                                      [data.k[3], data.k[4], data.k[5]],
                                      [data.k[6], data.k[7], data.k[8]]])
        self.distCoeffs = np.array([[i] for i in data.d])

    def static_tf_listener(self, parent, child):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        trans = None
        while not trans:
            try:
                trans = self.tf_buffer.lookup_transform(
                    child, parent, rclpy.time.Time())
            except Exception:  # noqa: E501
                pass
            rclpy.spin_once(self, timeout_sec=0.1)

        self.camera_translation = [
            trans.transform.translation.x, trans.transform.translation.y,
            trans.transform.translation.z
        ]
        self.camera_rotation = R.from_quat([
            trans.transform.rotation.x, trans.transform.rotation.y,
            trans.transform.rotation.z, trans.transform.rotation.w
        ])


def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
