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
import transforms3d
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster, TransformException, TransformListener
from tf2_ros.buffer import Buffer

TABLE_MARKERS = [20, 21, 22, 23]
COVARIANCE = [
    0.1, 0., 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0., 0., 0., 0.1, 0., 0., 0.,
    0., 0., 0., 0.1, 0., 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0., 0., 0., 0.1
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
            Image, '/camera/camera_central/RasPi0', self.image_listener_callback, 1)
        self.image_subscription
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/camera_central/RasPi0/camera_info',
            self.camera_info_listener_callback, 1)
        self.camera_info_subscription
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_listener('map', 'camera_prediction')
        self.br = CvBridge()

        self.dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.params = aruco.DetectorParameters_create()
        self.camera_matrix = None
        self.__map_camera_tf = None
        self.__map_camera_prediction_tf = None

    def _static_tf_listener(self, parent, child):
        """
        Get predicted camera and static ArUco 20-23 tag poses relative to map.

        In order to detect incorrect orientation of ArUco tags,
        it is necessary to have a prediction of the pose of the camera.
        map <- camera_prediction

        The predefined distances of ArUco 20-23 tag poses could also be assessed,
        if debug option is enabled.
        map <- marker_[20]_static
        map <- marker_[21]_static
        map <- marker_[22]_static
        map <- marker_[23]_static

        Variables tf_buffer and tf_listener need to be initialized in this function,
        otherwise the static transforms won't be assessed.

        TODO: add debug option
        """
        self.tf_buffer = Buffer()
        self._tf_listener = TransformListener(self.tf_buffer, self)
        trans = None
        while not trans:
            try:
                trans = self.tf_buffer.lookup_transform(
                    child, parent, rclpy.time.Time())
            except Exception:  # noqa: E501
                pass
            rclpy.spin_once(self, timeout_sec=0.1)

        transvec = [
            trans.transform.translation.x, trans.transform.translation.y,
            trans.transform.translation.z
        ]
        rotquat = [
            trans.transform.rotation.x, trans.transform.rotation.y,
            trans.transform.rotation.z, trans.transform.rotation.w
        ]
        tmat = np.eye(4)
        tmat[:3, :3] = transforms3d.quaternions.quat2mat(rotquat)
        tmat[:3, 3] = transvec
        self.__map_camera_prediction_tf = tmat

    def image_listener_callback(self, data):
        """
        Get the image and publish tf2 transforms of all ArUco tags.
        """
        if self.camera_matrix is None:
            return

        current_frame = self.br.imgmsg_to_cv2(data, 'bgr8')
        transformation_matrices, ids = self.get_aruco_pose(current_frame)
        self.find_map(transformation_matrices, ids)
        self.publish_transforms(transformation_matrices, ids)

    def camera_info_listener_callback(self, data):
        """
        Get camera matrix and distorsion coefficients of camera.
        """
        self.camera_matrix = np.array([[data.k[0], data.k[1], data.k[2]],
                                       [data.k[3], data.k[4], data.k[5]],
                                       [data.k[6], data.k[7], data.k[8]]])
        self.dist_coeffs = np.array([[i] for i in data.d])

        # TODO: Remove once https://github.com/cyberbotics/webots_ros2/pull/510 gets synced
        focal_length = (2 * self.camera_matrix[0][2]) / (2 *
                                                         math.tan(2.15 / 2))
        self.camera_matrix = np.array([[focal_length, 0, 1920 / 2],
                                       [0, focal_length, 1080 / 2], [0, 0, 1]])

    def get_aruco_pose(self, frame):
        """
        Get transformation matrices of all ArUco markers found in image.

        Table markers are larger (10 cm x 10 cm),
        while robot markers are smaller (7 cm x 7 cm).
        It is not possible to say beforehand which marker is which size,
        so estimatePoseSingleMarkers needs to be called several times,
        for the different ArUco marker sizes.
        The transformation matrices are later merged.
        """
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
                if ids[i] in TABLE_MARKERS:
                    tvecs[i] = tvecs_table[i]
                    rvecs[i] = rvecs_table[i]

            # TODO: show aruco pose only if debug mode is True
            self.show_aruco_pose(frame, corners, rvecs, tvecs, ids)

        transformation_matrices = self.create_transformation_matrices(
            tvecs, rvecs, len(ids))

        # TODO: if pose has bad orientation, remove here or in publish_transforms?
        return transformation_matrices, ids

    def create_transformation_matrices(self, tvecs, rvecs, len_ids):
        """
        Get transformation matrix from tvecs and rvecs.

        ArUco detection algoritm returns tvecs and rvecs.
        These values are not useful for further processing.
        Transform these values into transformation matrices.

        Further explanation:
        https://answers.opencv.org/question/161369/retrieve-yaw-pitch-roll-from-rvec/
        https://stackoverflow.com/questions/12933284/rodrigues-into-eulerangles-and-vice-versa
        """
        transformation_matrices = []

        for i in range(len_ids):
            # TODO: sve u rotmat, koristiti transforms3d
            translation = tvecs[i, 0]
            rmat = cv2.Rodrigues(rvecs[i])[0]

            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = rmat
            transformation_matrix[:3, 3] = translation
            transformation_matrices.append(transformation_matrix)
        return transformation_matrices

    def find_map(self, transformation_matrices, ids):
        """
        Find the position of the origin from table markers.

        We have only this information about the table position:
        camera <- marker_[20]
        camera <- marker_[21]
        camera <- marker_[22]
        camera <- marker_[23]
        It is necessary to find the origin (0,0,0) of the table.
        The origin should ideally be in the center of the rectangle
        formed by the four table marker poses.

        TODO: Make this function better!!
        TODO: what if some marker is missing
        TODO: future measurements should only correct the position of the map
        """
        # https://stackoverflow.com/questions/952914/how-do-i-make-a-flat-list-out-of-a-list-of-lists
        ids_flattened = [item for sublist in ids for item in sublist]
        if 20 not in ids_flattened:
            return
        camera_marker20_tf = transformation_matrices[ids_flattened.index(20)]

        # TODO: ne zelim da mi ovde stoje informacije o pozicijama na terenu
        map_marker20_tf = np.eye(4)
        map_marker20_tf[:3, 3] = np.array([-0.430, 0.925, 0])

        if camera_marker20_tf is not None:
            camera_map_tf = camera_marker20_tf @ np.linalg.inv(map_marker20_tf)
            self.__map_camera_tf = np.linalg.inv(camera_map_tf)
            self.publish_transform('map', 'camera', self.__map_camera_tf)

    def check_alignment(self, tmat, axis):
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

        Elaboration on problem:
        https://github.com/opencv/opencv/issues/8813
        """
        return True
        # TODO: biti konzistentan, koristiti svuda ili tmat ili transformation_matrix
        # TODO: testirati da li radi izmenjena funkcija, pomerati robote
        print("tmat")
        print(tmat)
        rotmat = tmat[:3, :3]
        rotmat_camera = self.__map_camera_prediction_tf[:3, :3]
        print("rotmat_camera")
        print(rotmat_camera)
        dot_product = np.dot(rotmat * axis, rotmat_camera * axis)
        if dot_product > 0.707:
            return True
        return False

    def publish_transforms(self, transformation_matrices, ids):
        """
        Publish all poses in the tf2 tree.

        All poses are published with map as parent.

        self.__map_camera_tf: map <- camera
        tmat: camera <- marker
        @: map <- marker
        """
        if ids is not None:
            for i in range(len(ids)):
                tmat = transformation_matrices[i]

                # TODO: if debug enable raw_marker
                # self.publish_transform('map', f'raw_marker_{ids[i]}',
                #                        self.__map_camera_tf @ tmat)

                if ids[i] in TABLE_MARKERS:
                    if self.check_alignment(
                            tmat, [0, 1, 0]) and self.check_alignment(
                                tmat, [0, 0, 1]):
                        self.publish_transform('map', f'marker_{ids[i]}',
                                               self.__map_camera_tf @ tmat)
                else:
                    if self.check_alignment(tmat, [0, 0, 1]):
                        self.publish_transform('map', f'marker_{ids[i]}',
                                               self.__map_camera_tf @ tmat)

    def publish_transform(self, frame_id, child_frame_id,
                          transformation_matrix):
        """
        Publish the transformation to tf2.
        """
        translation = transformation_matrix[:3, 3]
        rotation = transforms3d.quaternions.mat2quat(
            transformation_matrix[:3, :3])
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = frame_id
        transform_stamped.child_frame_id = child_frame_id
        transform_stamped.transform.translation.x = translation[0]
        transform_stamped.transform.translation.y = translation[1]
        transform_stamped.transform.translation.z = translation[2]
        transform_stamped.transform.rotation.x = rotation[1]
        transform_stamped.transform.rotation.y = rotation[2]
        transform_stamped.transform.rotation.z = rotation[3]
        transform_stamped.transform.rotation.w = rotation[0]
        self._tf_broadcaster.sendTransform(transform_stamped)

    def show_aruco_pose(self, frame, corners, rvecs, tvecs, ids):
        """
        Emphasize detected ArUco markers on video or image frame.
        """
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
