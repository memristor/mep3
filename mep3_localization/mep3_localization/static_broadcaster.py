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

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.

    This node publishes transforms:
    - from map origin to the `marker_[42]` tag
    - and from map origin to the camera.
    The transforms are only published once at startup, and are constant for all
    time.
    """

    def __init__(self):
        super().__init__('static_turtle_tf2_broadcaster')

        self._tf_publisher = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms('marker_[42]', 'map', [0.0, 0.205, 0.0],
                             [0.0, 0.0, 0.0, 1.0])
        self.make_transforms('map', 'camera_static', [-0.141, 1.212, 1.184],
                             [-0.005, 0.962, -0.272, 0.008])

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

    def make_inverse_transforms(self, frame_id, child_frame_id, translation,
                                rotation):
        translation = [-i for i in translation]
        r = R.from_quat(rotation)
        rotation = r.inv().as_quat()
        self.make_transforms(child_frame_id, frame_id, translation, rotation)


def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
