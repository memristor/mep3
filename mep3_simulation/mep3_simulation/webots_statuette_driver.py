from math import radians

import rclpy
from rclpy.executors import MultiThreadedExecutor


class WebotsStatuetteDriver:

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:  # noqa: E501
            pass  # noqa: E501
        self.__executor = MultiThreadedExecutor()

        self.__robot = webots_node.robot
        timestep = int(self.__robot.getBasicTimeStep())
        self.__connector = self.__robot.getDevice('hand_L_statuette_connector')
        self.__connector.enablePresence(timestep)
        self.__motor = self.__robot.getDevice('hand_mid_L')
        self.__encoder = self.__motor.getPositionSensor()
        self.__encoder.enable(timestep)
        self.__node = rclpy.create_node('webots_statuette_driver_node')
        self.__destroy_node = False
        self.__enable_statuette_decoupling = False
        self.__REPLICA_DECOUPLING_ANGLE = radians(80)
        self.__STATUETTE_DECOUPLING_ANGLE = radians(5)

    def step(self):
        """
        Check in every time step if statuette should be decoupled.

        In order to decouple the statuette, we first need to decouple the
        replica. We have a three state FSM:
        coupled_statuette -> decoupled_replica -> decoupled_statuette.
        """
        if self.__destroy_node:
            return
        if self.__connector.getPresence() and not self.__connector.isLocked():
            # state: coupled_statuette
            self.__connector.lock()
        elif self.__encoder.getValue() > self.__REPLICA_DECOUPLING_ANGLE:
            self.__enable_statuette_decoupling = True
        elif self.__encoder.getValue(
        ) <= self.__STATUETTE_DECOUPLING_ANGLE and self.__connector.isLocked(
        ) and self.__enable_statuette_decoupling:
            self.__connector.unlock()
            self.__destroy_node = True
            self.__node.destroy_node()
        rclpy.spin_once(self.__node, timeout_sec=0, executor=self.__executor)
