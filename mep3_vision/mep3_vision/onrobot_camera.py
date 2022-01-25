# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import cv2  # OpenCV library
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import os
import yaml
import numpy as np


class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')

        # TODO: Calibration. Now it's just the Dummy calibration file...
        if True:
            with open("src/mep3/mep3_vision/mep3_vision/ost.yaml", "r") as stream:
            # with open("ost.yaml", "r") as stream:
                try:
                    camera_data = yaml.safe_load(stream)
                except yaml.YAMLError as exc:
                    print(exc)

                self.cameraMatrix = np.array(camera_data['camera_matrix']['data'])
                self.distCoeffs = np.array(camera_data['distortion_coefficients']['data'])
                self.cameraMatrix = self.cameraMatrix.reshape(3, 3)
                self.distCoeffs = self.distCoeffs.reshape(1, 5)

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image,
            '/big/MemristorRobot/camera',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Aruco detection
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(current_frame, arucoDict,
                                                           parameters=arucoParams)

        if ids is not None and len(ids) > 0:
            # Estimate the posture per each Aruco marker


            rotation_vectors, translation_vectors, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 1,
                                                                                                    self.cameraMatrix,
                                                                                                    self.distCoeffs)

            cv2.aruco.drawDetectedMarkers(current_frame, corners, ids)
            i = 0
            for rvec, tvec in zip(rotation_vectors, translation_vectors):
                cv2.aruco.drawAxis(current_frame, self.cameraMatrix, self.distCoeffs, rvec, tvec, 1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                # cv2.putText(current_frame, 'Christmas', (int(corners[i][0][0][0], int(corners[i][0][0][1]))),
                #             font, 4,(255,255,255),2,cv2.LINE_AA)
                print((int(corners[i][0][0][0]), int(corners[i][0][0][1])))
                cv2.putText(current_frame, "{:.3f} {:.3f}".format(translation_vectors[0][0][:2][0],
                                                                  translation_vectors[0][0][:2][1]
                                                                  ), (int(corners[i][0][0][0]), int(corners[i][0][0][1])),
                            font, 1, (0, 128, 128), 2, cv2.LINE_AA)

                i+= 1
                print("Translation vector: ", tvec)

        # Display image
        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()