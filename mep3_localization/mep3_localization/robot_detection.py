#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv2 import aruco
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

import tf_transformations
from geometry_msgs.msg import TransformStamped
"""
This class receives video from the central RasPi Cam,
and sends TF of Aruco tags.
"""


class DetectedRobots(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image, '/cam/cam_central/RasPi0', self.listener_callback, 10)
        self.subscription
        self._tf_publisher = TransformBroadcaster(self)

        # cv_file = cv2.FileStorage('calibration_camera.yaml', cv2.FILE_STORAGE)

        self.br = CvBridge()
        self.dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.params = aruco.DetectorParameters_create()
        # self.cameraMatrix = cv_file.getNode('K').mat()
        # self.distCoeffs = cv_file.getNode('D').mat()
        self.cameraMatrix = np.array([[570.34, 0, 1920 / 2],
                                      [0, 570.34, 1080 / 2], [0, 0, 1]])
        self.distCoeffs = np.zeros((5, 1))
        # Side length of the ArUco marker in meters
        self.aruco_robot_length = 0.07
        self.aruco_map_length = 0.1
        # cv_file.release()

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        self.draw_aruco_pose(current_frame)

    def make_transforms(self, tvec, rvec, id):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'world'
        static_transformStamped.child_frame_id = 'robot' + str(id)
        static_transformStamped.transform.translation.x = tvec[0, 0]
        static_transformStamped.transform.translation.y = tvec[0, 1]
        static_transformStamped.transform.translation.z = tvec[0, 2]

        # https://answers.opencv.org/question/161369/retrieve-yaw-pitch-roll-from-rvec/
        # https://stackoverflow.com/questions/12933284/rodrigues-into-eulerangles-and-vice-versa
        rmat = cv2.Rodrigues(rvec)[0]
        r = R.from_matrix(rmat)
        quat = r.as_quat()
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._tf_publisher.sendTransform(static_transformStamped)

    def draw_aruco_pose(self, frame):
        (corners, ids,
         rejected) = aruco.detectMarkers(frame,
                                         self.dict,
                                         parameters=self.params,
                                         cameraMatrix=self.cameraMatrix,
                                         distCoeff=self.distCoeffs)
        # Check that at least one ArUco marker was detected
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.aruco_robot_length, self.cameraMatrix,
                self.distCoeffs)
            rvecs_map, tvecs_map, obj_points_map = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.aruco_map_length, self.cameraMatrix,
                self.distCoeffs)

            for i in range(len(rvecs)):
                if ids[i] <= 10:
                    aruco.drawAxis(frame, self.cameraMatrix, self.distCoeffs,
                                   rvecs[i], tvecs[i], self.aruco_robot_length)
                    self.make_transforms(tvecs[i], rvecs[i], ids[i])
                elif ids[i] == 42:
                    aruco.drawAxis(frame, self.cameraMatrix, self.distCoeffs,
                                   rvecs_map[i], tvecs_map[i],
                                   self.aruco_map_length)
                    self.make_transforms(tvecs_map[i], rvecs_map[i], ids[i])
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

    def draw_aruco_border(self, frame):
        (corners, ids, rejected) = aruco.detectMarkers(frame,
                                                       self.dict,
                                                       parameters=self.params)
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                # draw the ArUco marker ID on the frame
                cv2.putText(frame, str(markerID),
                            (topLeft[0], topLeft[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("camera", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    detected_robots = DetectedRobots()
    rclpy.spin(detected_robots)
    detected_robots.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
