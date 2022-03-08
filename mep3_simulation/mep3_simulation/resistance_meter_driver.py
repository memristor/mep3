from controller import Supervisor

import rclpy
from std_msgs.msg import Int8

class ResistanceMeterDriver:

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:  # noqa: E501
            # logging.exception("WaitMatchStartDriver")
            pass  # noqa: E501

        self.__robot = webots_node.robot
        self.__touch_sensor_left = self.__robot.getDevice('hand_left_Dz_touch_sensor')
        self.__touch_sensor_right = self.__robot.getDevice('hand_right_Dz_touch_sensor')

        self.__node = rclpy.node.Node('webots_resistance_meter_driver')
        self.__publisher = self.__node.create_publisher(Int8, '/resistance_meter', 1)


    def step(self):

        root = self.__robot.getSelf()

        print(root.getField('translation').getSFVec3f())

        # self.__publisher.publish(Int8(data=69))
        rclpy.spin_once(self.__node, timeout_sec=0)
