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
from tf2_ros import TransformBroadcaster, TransformListener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer


class PoseCorrector(Node):
    """
    Corrects pose estimation of ArUco tags.

    This class receives video and camera parameters from the central RasPi Cam,
    receives the static transform from map to camera in order to detect wrong
    orientations and sends TF of Aruco tags.
    """

    def __init__(self):
        super().__init__('pose_corrector')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        from_frame_rel = 'raw_camera'
        to_frame_rel = 'raw_marker_[20]'
        try:
            t = self._tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        self.send_transform_stamped(t)

    def send_transform_stamped(self, t):
        msg = TransformStamped()
        msg = t
        msg.header.frame_id = 'camera_static'
        msg.child_frame_id = 'raw_marker_[20]_test'
        self._tf_broadcaster.sendTransform(msg)

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


def main(args=None):
    rclpy.init(args=args)
    pose_corrector = PoseCorrector()
    rclpy.spin(pose_corrector)
    pose_corrector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
