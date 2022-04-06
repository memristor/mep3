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
from tf2_ros.transform_listener import TransformListener


class RobotDetector(Node):
    """
    Detects robots from the central tracking device.

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
        self.static_tf_listener('map', 'camera_static')
        self.__cv_bridge = CvBridge()

        self.__dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.__params = aruco.DetectorParameters_create()
        # Side length of the ArUco marker in meters
        self.__aruco_robot_length = 0.07
        self.__aruco_42_length = 0.1
        self.__ARUCO_NUMBER_42 = 42
        self.__ARUCO_NUMBER_ROBOTS_MAX = 10

    def image_listener_callback(self, data):
        current_frame = self.__cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        rvecs, tvecs, ids = self.get_aruco_pose(current_frame)
        self.send_pose_tf2(rvecs, tvecs, ids)

    def static_tf_listener(self, parent, child):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        trans = None
        while not trans:
            try:
                trans = self.tf_buffer.lookup_transform(
                    child, parent, rclpy.time.Time())
            except Exception:  # noqa: E501
                # logging.exception("StaticTfListener"):
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

    def camera_info_listener_callback(self, data):
        self.__camera_matrix = np.array([[data.k[0], data.k[1], data.k[2]],
                                         [data.k[3], data.k[4], data.k[5]],
                                         [data.k[6], data.k[7], data.k[8]]])
        self.__dist_coeffs = np.array([[i] for i in data.d])

    def send_pose_tf2(self, rvecs, tvecs, ids):
        if ids is not None:
            for i in range(len(rvecs)):
                if ids[i] < self.__ARUCO_NUMBER_ROBOTS_MAX or ids[
                        i] == self.__ARUCO_NUMBER_42:
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
        """
        Arrow convention: parent <- child.

        We know beforehand the following transformations:
        camera <- map,
        map <- marker_[42].
        We also know rotation(map) == rotation(marker_[42]).

        If we apply the camera <- map transformation on the [0, 0, 1] vector,
        then the z-axes of the transformation and the marker_[42] should be
        collinear. Since all markers have collinear z-axes (facing up from the
        table), the same applies for all other markers.

        Additionally, we know that the x- and y-axes of marker_[42] should be
        collinear with x- and y-axes of the camera <- map transformation.

        The dot product is used to check if vectors are collinear. If the dot
        product is larger than a threshold, we assume they are collinear.
        The threshold is currently set at 1/sqrt(2).
        """
        dot_product = np.dot(r.apply(axis),
                             self.camera_rotation.inv().apply(axis))
        # print(dot_product)
        if dot_product > 0.707:
            return True
        return False

    def make_transforms(self, frame_id, child_frame_id, translation, rotation):
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = frame_id
        static_transform_stamped.child_frame_id = child_frame_id
        static_transform_stamped.transform.translation.x = translation[0]
        static_transform_stamped.transform.translation.y = translation[1]
        static_transform_stamped.transform.translation.z = translation[2]
        static_transform_stamped.transform.rotation.x = rotation[0]
        static_transform_stamped.transform.rotation.y = rotation[1]
        static_transform_stamped.transform.rotation.z = rotation[2]
        static_transform_stamped.transform.rotation.w = rotation[3]
        self._tf_publisher.sendTransform(static_transform_stamped)

    def get_aruco_pose(self, frame):
        (corners, ids,
         rejected) = aruco.detectMarkers(frame,
                                         self.__dict,
                                         parameters=self.__params,
                                         cameraMatrix=self.__camera_matrix,
                                         distCoeff=self.__dist_coeffs)
        tvecs = rvecs = None
        if ids is not None:
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.__aruco_robot_length, self.__camera_matrix,
                self.__dist_coeffs)
            rvecs_42, tvecs_42, obj_points_42 = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.__aruco_42_length, self.__camera_matrix,
                self.__dist_coeffs)

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
                aruco.drawAxis(frame, self.__camera_matrix, self.__dist_coeffs,
                               rvecs[i], tvecs[i], self.__aruco_robot_length)
            elif ids[i] == 42:
                aruco.drawAxis(frame, self.__camera_matrix, self.__dist_coeffs,
                               rvecs[i], tvecs[i], self.__aruco_42_length)
        cv2.imshow('camera', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    robot_detector = RobotDetector()
    rclpy.spin(robot_detector)
    robot_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
