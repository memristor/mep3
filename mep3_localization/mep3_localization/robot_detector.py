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

import numpy as np
import rclpy
from rclpy.node import Node


class RobotDetector(Node):

    def __init__(self):
        super().__init__('robot_detector')

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


def main(args=None):
    rclpy.init(args=args)
    robot_detector = RobotDetector()
    rclpy.spin(tobot_detector)
    robot_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
