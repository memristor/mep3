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
# TODO: create __append_covariance procedure
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

    Arrow convention for tf transformations: parent <- child.
    """

    def __init__(self):
        super().__init__('aruco_detector')

        self.__camera_matrix = None
        self.__map_camera_tf = None
        self.__map_camera_prediction_tf = None
        self.__map_marker_20_static_tf = None
        self.__map_marker_21_static_tf = None
        self.__map_marker_22_static_tf = None
        self.__map_marker_23_static_tf = None
        self.__dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.__params = aruco.DetectorParameters_create()
        self.__br = CvBridge()

        self.declare_parameter('debug', False)
        self.__debug = self.get_parameter(
            'debug').get_parameter_value().bool_value

        self.__image_subscription = self.create_subscription(
            Image, '/camera/camera_central/RasPi0', self.__image_callback, 1)
        self.__image_subscription
        self.__camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/camera_central/RasPi0/camera_info',
            self.__camera_info_callback, 1)
        self.__camera_info_subscription
        self.__tf_broadcaster = TransformBroadcaster(self)
        # pwcs = PoseWithCovarianceStamped
        # TODO: define as parameter ArUco number of robot, also in ekf.yaml
        self.__pwcs_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/camera/aruco_5', 1)

        # Use debug mode if you want to plot these transforms in RViz.
        # __static_tf_callback() is not used in order to increase startup speed.
        if self.__debug:
            self.__static_tf_callback('map', 'camera_prediction')
            self.__static_tf_callback('map', 'marker_[20]_static')
            self.__static_tf_callback('map', 'marker_[21]_static')
            self.__static_tf_callback('map', 'marker_[22]_static')
            self.__static_tf_callback('map', 'marker_[23]_static')
        else:
            self.__map_camera_prediction_tf = [
                [-1.0e+00, 8.70048930e-06, -5.07844365e-06, 0],
                [1.00740663e-05, 8.66028649e-01, -4.99994379e-01, 1.50976e+00],
                [4.78819448e-08, -4.99994379e-01, -8.66028649e-01, 1.05e+00],
                [0.0e+00, 0.0e+00, 0.0e+00, 1.0e+00]
            ]

            self.__map_marker_20_static_tf = [[1., 0., 0., -0.43],
                                              [0., 1., 0., 0.925],
                                              [0., 0., 1., 0.],
                                              [0., 0., 0., 1.]]

            self.__map_marker_21_static_tf = [[1., 0., 0., 0.43],
                                              [0., 1., 0., 0.925],
                                              [0., 0., 1., 0.],
                                              [0., 0., 0., 1.]]

            self.__map_marker_22_static_tf = [[1., 0., 0., -0.43],
                                              [0., 1., 0., -0.925],
                                              [0., 0., 1., 0.],
                                              [0., 0., 0., 1.]]

            self.__map_marker_23_static_tf = [[1., 0., 0., 0.43],
                                              [0., 1., 0., -0.925],
                                              [0., 0., 1., 0.],
                                              [0., 0., 0., 1.]]

    def __static_tf_callback(self, parent, child):
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
        """
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)
        transform = None
        while not transform:
            try:
                transform = self.__tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time())
            except Exception:  # noqa: E501
                pass
            rclpy.spin_once(self, timeout_sec=0.1)

        translation = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ]
        quaternion = [
            transform.transform.rotation.w, transform.transform.rotation.x,
            transform.transform.rotation.y, transform.transform.rotation.z
        ]
        tmat = np.eye(4)
        tmat[:3, :3] = transforms3d.quaternions.quat2mat(quaternion)
        tmat[:3, 3] = translation
        if child == 'camera_prediction':
            self.__map_camera_prediction_tf = tmat
        elif child == "marker_[20]_static":
            self.__map_marker_20_static_tf = tmat
        elif child == "marker_[21]_static":
            self.__map_marker_21_static_tf = tmat
        elif child == "marker_[22]_static":
            self.__map_marker_22_static_tf = tmat
        elif child == "marker_[23]_static":
            self.__map_marker_23_static_tf = tmat

    def __image_callback(self, data):
        """
        Get the image and publish tf2 transforms of all ArUco tags.
        """
        if self.__camera_matrix is None:
            return

        current_frame = self.__br.imgmsg_to_cv2(data, 'bgr8')
        transformation_matrices, ids = self.__aruco_pose_getter(current_frame)
        self.__find_map(transformation_matrices, ids)
        self.__publish_tmats(transformation_matrices, ids)

    def __camera_info_callback(self, data):
        """
        Get camera matrix and distorsion coefficients of camera.
        """
        self.__camera_matrix = np.array([[data.k[0], data.k[1], data.k[2]],
                                         [data.k[3], data.k[4], data.k[5]],
                                         [data.k[6], data.k[7], data.k[8]]])
        self.dist_coeffs = np.array([[i] for i in data.d])

        # TODO: Remove once https://github.com/cyberbotics/webots_ros2/pull/510 gets synced
        focal_length = (2 * self.__camera_matrix[0][2]) / (2 *
                                                           math.tan(2.15 / 2))
        self.__camera_matrix = np.array([[focal_length, 0, 1920 / 2],
                                         [0, focal_length, 1080 / 2],
                                         [0, 0, 1]])

        #TODO: get instead of getter
    def __aruco_pose_getter(self, frame):
        """
        Get transformation matrices of all ArUco markers found in image.

        Table markers are larger (10 cm x 10 cm),
        while robot markers are smaller (7 cm x 7 cm).
        It is not possible to say beforehand which marker is which size,
        so estimatePoseSingleMarkers needs to be called several times,
        for the different ArUco marker sizes.
        The transformation matrices are later merged.
        """
        corners, ids, _ = aruco.detectMarkers(
            frame,
            self.__dict,
            parameters=self.__params,
            cameraMatrix=self.__camera_matrix,
            distCoeff=self.dist_coeffs)
        tvecs = rvecs = None
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, ARUCO_ROBOT_TAG_LENGTH, self.__camera_matrix,
                self.dist_coeffs)
            rvecs_table, tvecs_table, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, ARUCO_TABLE_TAG_LENGTH, self.__camera_matrix,
                self.dist_coeffs)

            for i in range(len(rvecs)):
                if ids[i] in TABLE_MARKERS:
                    tvecs[i] = tvecs_table[i]
                    rvecs[i] = rvecs_table[i]

            if self.__debug:
                self.__show_aruco_pose(frame, corners, rvecs, tvecs, ids)

        transformation_matrices = self.__create_transformation_matrices(
            tvecs, rvecs, len(ids))

        return transformation_matrices, ids

    def __create_transformation_matrices(self, tvecs, rvecs, len_ids):
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
            translation = tvecs[i, 0]
            rmat = cv2.Rodrigues(rvecs[i])[0]

            tmat = np.eye(4)
            tmat[:3, :3] = rmat
            tmat[:3, 3] = translation
            transformation_matrices.append(tmat)
        return transformation_matrices

    def __find_map(self, transformation_matrices, ids):
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

        Currently, position of map is calculated based solely on the position
        of marker_[20]:
        map <- camera = map <- marker_[20]_static @ inverse(camera <- marker_[20])

        TODO: Make this function better more sophisticated
        TODO: what if some marker is missing? E.g. marker_[20]
        TODO: future measurements should only correct the position of the map
        """
        # https://stackoverflow.com/questions/952914/how-do-i-make-a-flat-list-out-of-a-list-of-lists
        ids_flattened = [item for sublist in ids for item in sublist]
        if 20 not in ids_flattened:
            return
        camera_marker_20_tf = transformation_matrices[ids_flattened.index(20)]

        if camera_marker_20_tf is not None:
            camera_map_tf = camera_marker_20_tf @ np.linalg.inv(
                self.__map_marker_20_static_tf)
            self.__map_camera_tf = np.linalg.inv(camera_map_tf)
            self.__publish_tmat('map', 'camera', self.__map_camera_tf)

    def __check_alignment(self, tmat, axis):
        """
        Check if predicted map orientation is collinear with ArUco tag orientation.

        tmat: (camera <- marker_[x]) transformation matrix
        axis: [1, 0, 0] or [0, 0, 1]

        We know beforehand the following transformations:
        map <- camera_prediction,
        map <- marker_[20]_static.
        map <- marker_[21]_static.
        map <- marker_[22]_static.
        map <- marker_[23]_static.

        We also know:
        rotation(map) == rotation(marker_[20]_static),
        rotation(map) == rotation(marker_[21]_static),
        rotation(map) == rotation(marker_[22]_static),
        rotation(map) == rotation(marker_[23]_static),

        All ArUco tags should have the z-axes facing up from the table,
        while map orientation and ArUco tags on the table (20, 21, 22 and 23)
        should have x-axes facing to the back side of the table
        and y-axes facing to the left side of the table.
        Comparing x- and z-axis would be sufficient to say
        that y-axis is also collinear.

        In other words, these rules should always be true:
        z_axis_map == z_axis_marker_[x]
        x_axis_map == x_axis_marker_[20]
        x_axis_map == x_axis_marker_[21]
        x_axis_map == x_axis_marker_[22]
        x_axis_map == x_axis_marker_[23]

        If we consider the camera to be the origin of our frame,
        we can predict the position of map by applying:
        map_prediction = inverse(map <- camera_prediction).

        To get the x- and z-axis unit vector of map_prediction from camera frame,
        we apply:
        x_axis_map = inverse(map <- camera_prediction) @ [1, 0, 0].
        z_axis_map = inverse(map <- camera_prediction) @ [0, 0, 1].

        To get the z-axis unit vector of any ArUco marker from camera frame,
        and the x-axis unit vector of table ArUco markers, we apply:
        z_axis_marker_[x] = (camera <- marker_[x]) @ [0, 0, 1].
        x_axis_marker_[20] = (camera <- marker_[20]) @ [1, 0, 0].
        x_axis_marker_[21] = (camera <- marker_[21]) @ [1, 0, 0].
        x_axis_marker_[22] = (camera <- marker_[22]) @ [1, 0, 0].
        x_axis_marker_[23] = (camera <- marker_[23]) @ [1, 0, 0].

        The dot product is used to check if vectors are close to collinear.
        Real world measurements would never return perfectly parallel vectors.
        If the dot product is larger than a threshold, we assume they are collinear.
        The threshold is currently set at 1/sqrt(2), e.g.:

        If dot(z_axis_marker_[x], z_axis_map) > 1/sqrt(2) == True - they are collinear.

        Elaboration on problem:
        https://github.com/opencv/opencv/issues/8813

        TODO: if __find_map() becomes more accurate, replace
              (map <- camera_prediction) with (map <- camera)
        """
        rotmat_camera_marker = tmat[:3, :3]
        rotmat_camera_map_prediction = np.linalg.inv(
            self.__map_camera_prediction_tf)[:3, :3]
        dot_product = np.dot(rotmat_camera_marker @ axis,
                             rotmat_camera_map_prediction @ axis)
        if dot_product > 0.707:
            return True
        return False

    def __publish_tmats(self, transformation_matrices, ids):
        """
        Publish all poses in the tf2 tree.

        All poses are published with map as parent.

        self.__map_camera_tf: map <- camera
        tmat: camera <- marker
        @: map <- marker

        If debug mode is True, show incorrectly oriented markers.
        """
        if ids is not None:
            for i in range(len(ids)):
                tmat = transformation_matrices[i]

                if ids[i] in TABLE_MARKERS:
                    if self.__check_alignment(
                            tmat, [1, 0, 0]) and self.__check_alignment(
                                tmat, [0, 0, 1]):
                        self.__publish_tmat('map', f'marker_{ids[i]}',
                                            self.__map_camera_tf @ tmat)
                    elif self.__debug:
                        self.__publish_tmat('map',
                                            f'marker_{ids[i]}_incorrect',
                                            self.__map_camera_tf @ tmat)

                else:
                    if self.__check_alignment(tmat, [0, 0, 1]):
                        self.__publish_tmat('map', f'marker_{ids[i]}',
                                            self.__map_camera_tf @ tmat)
                    elif self.__debug:
                        self.__publish_tmat('map',
                                            f'marker_{ids[i]}_incorrect',
                                            self.__map_camera_tf @ tmat)

    def __publish_tmat(self, frame_id, child_frame_id, tmat):
        """
        Publish the transformation matrix to tf2.
        """
        translation = tmat[:3, 3]
        rotation = transforms3d.quaternions.mat2quat(tmat[:3, :3])
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
        self.__tf_broadcaster.sendTransform(transform_stamped)

        pwcs = PoseWithCovarianceStamped()
        pwcs.header = transform_stamped.header
        pwcs.pose.pose.position.x = translation[0]
        pwcs.pose.pose.position.y = translation[1]
        pwcs.pose.pose.position.z = translation[2]
        pwcs.pose.pose.orientation = transform_stamped.transform.rotation
        pwcs.pose.covariance = COVARIANCE
        self.__pwcs_publisher.publish(pwcs)

    def __show_aruco_pose(self, frame, corners, rvecs, tvecs, ids):
        """
        Emphasize detected ArUco markers on video or image frame.
        """
        aruco.drawDetectedMarkers(frame, corners, ids)
        for i in range(len(rvecs)):
            if ids[i] <= 10:
                aruco.drawAxis(frame, self.__camera_matrix, self.dist_coeffs,
                               rvecs[i], tvecs[i], ARUCO_ROBOT_TAG_LENGTH)
            elif ids[i] in TABLE_MARKERS:
                aruco.drawAxis(frame, self.__camera_matrix, self.dist_coeffs,
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
