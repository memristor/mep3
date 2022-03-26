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

    def step(self):
        """
        If the angle of the motor is above a threshold,
        release the statuette.
        The threshold is currently set at 80 degrees.
        """
        if self.__connector.getPresence() and not self.__connector.isLocked():
            self.__connector.lock()
        elif self.__encoder.getValue() < 0 and self.__connector.isLocked():
            self.__connector.unlock()
            self.__destroy_node = True

        if self.__destroy_node:
            self.__node.destroy_node()
        else:
            rclpy.spin_once(self.__node,
                            timeout_sec=0,
                            executor=self.__executor)
