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
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import numpy as np
import rclpy
import math
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster, TransformException, TransformListener
from tf2_ros.buffer import Buffer


TABLE_MARKERS = [20, 21, 22, 23]
COVARIANCE = [0.1, 0., 0., 0., 0., 0.,
              0., 0.1, 0., 0., 0., 0.,
              0., 0., 0.1, 0., 0., 0.,
              0., 0., 0., 0.1, 0., 0.,
              0., 0., 0., 0., 0.1, 0.,
              0., 0., 0., 0., 0., 0.1
              ]
ARUCO_ROBOT_TAG_LENGTH = 0.07  # meters
ARUCO_TABLE_TAG_LENGTH = 0.1  # meters


class ArucoDetector(Node):
    """
    Detects Aruco tags from the central tracking device.

    This class receives video and camera parameters from the central RasPi Cam,
    receives the static transform from map to camera in order to detect wrong
    orientations and sends TF of Aruco tags.
    """

    def __init__(self):
        super().__init__('aruco_detector')
        self.image_subscription = self.create_subscription(
            Image, '/cam/cam_central/RasPi0', self.image_listener_callback, 1)
        self.image_subscription
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/cam/cam_central/RasPi0/camera_info',
            self.camera_info_listener_callback, 1)
        self.camera_info_subscription
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_listener('map', 'camera')
        self.br = CvBridge()
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'aruco/marker_20', 1)

        self.dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.params = aruco.DetectorParameters_create()

    def _static_tf_listener(self, parent, child):
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self)
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

    def send_relative_robot_markers(self):
        # TODO: find position of map in raw aruco marker frame
        from_frame_rel = f'raw_marker_[20]_test'
        to_frame_rel = f'map'
        try:
            t = self._tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        self.send_transform_stamped(
            t, 'camera_static', from_frame_rel + '_test')
        self._publisher.publish(self.ts2pwcs(t))

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
        if dot_product > 0.707:
            return True
        return False

    def image_listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        rvecs, tvecs, ids = self.get_aruco_pose(current_frame)
        self.publish_transforms(rvecs, tvecs, ids)

    def camera_info_listener_callback(self, data):
        self.camera_matrix = np.array([[data.k[0], data.k[1], data.k[2]],
                                       [data.k[3], data.k[4], data.k[5]],
                                       [data.k[6], data.k[7], data.k[8]]])
        self.dist_coeffs = np.array([[i] for i in data.d])

        # TODO: Change once https://github.com/cyberbotics/webots_ros2/pull/510 gets synced
        focal_length = (
            2 * self.camera_matrix[0][2]) / (2 * math.tan(2.15 / 2))
        self.camera_matrix = np.array([[focal_length, 0, 1920 / 2],
                                      [0, focal_length, 1080 / 2], [0, 0, 1]])

    def get_aruco_pose(self, frame):
        corners, ids, _ = aruco.detectMarkers(frame,
                                              self.dict,
                                              parameters=self.params,
                                              cameraMatrix=self.camera_matrix,
                                              distCoeff=self.dist_coeffs)
        tvecs = rvecs = None
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, ARUCO_ROBOT_TAG_LENGTH, self.camera_matrix,
                self.dist_coeffs)
            rvecs_table, tvecs_table, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, ARUCO_TABLE_TAG_LENGTH, self.camera_matrix,
                self.dist_coeffs)

            for i in range(len(rvecs)):
                # Table markers are TAG_20, TAG_21, TAG_22 and TAG_23
                if ids[i] in TABLE_MARKERS:
                    tvecs[i] = tvecs_table[i]
                    rvecs[i] = rvecs_table[i]

            self.show_aruco_pose(frame, corners, rvecs, tvecs, ids)

        return rvecs, tvecs, ids

    def publish_transforms(self, rvecs, tvecs, ids):
        if ids is not None:
            for i in range(len(rvecs)):
                translation = tvecs[i, 0]
                # https://answers.opencv.org/question/161369/retrieve-yaw-pitch-roll-from-rvec/
                # https://stackoverflow.com/questions/12933284/rodrigues-into-eulerangles-and-vice-versa
                # https://github.com/opencv/opencv/issues/8813
                rmat = cv2.Rodrigues(rvecs[i])[0]
                r = R.from_matrix(rmat)
                rotation = r.as_quat()

                self.publish_transform('camera', f'raw_marker_{ids[i]}',
                                       translation, rotation)

                if ids[i] in TABLE_MARKERS:
                    if self.check_alignment(
                            r, [0, 1, 0]) and self.check_alignment(
                                r, [0, 0, 1]):
                        self.publish_transform('camera', f'marker_{ids[i]}',
                                             translation, rotation)
                else:
                    if self.check_alignment(r, [0, 0, 1]):
                        self.publish_transform('camera', f'marker_{ids[i]}',
                                             translation, rotation)

    def publish_transform(self, frame_id, child_frame_id, translation, rotation):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = frame_id
        transform_stamped.child_frame_id = child_frame_id
        transform_stamped.transform.translation.x = translation[0]
        transform_stamped.transform.translation.y = translation[1]
        transform_stamped.transform.translation.z = translation[2]
        transform_stamped.transform.rotation.x = rotation[0]
        transform_stamped.transform.rotation.y = rotation[1]
        transform_stamped.transform.rotation.z = rotation[2]
        transform_stamped.transform.rotation.w = rotation[3]
        self._tf_broadcaster.sendTransform(transform_stamped)

    def show_aruco_pose(self, frame, corners, rvecs, tvecs, ids):
        aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(rvecs)):
            if ids[i] <= 10:
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs,
                               rvecs[i], tvecs[i], ARUCO_ROBOT_TAG_LENGTH)
            elif ids[i] in TABLE_MARKERS:
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs,
                               rvecs[i], tvecs[i], ARUCO_TABLE_TAG_LENGTH)
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
