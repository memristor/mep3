import rclpy
from std_msgs.msg import Int32

SAMPLING_INTERVAL = 0.5 # seconds

class ResistanceMeterDriver:

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:  # noqa: E501
            # logging.exception("WaitMatchStartDriver")
            pass  # noqa: E501

        self.__supervisor = webots_node.robot
        self.__robot = self.__supervisor.getSelf()

        self.__touch_sensor_left = self.__supervisor.getDevice('hand_left_Dz_touch_sensor')
        self.__touch_sensor_right = self.__supervisor.getDevice('hand_right_Dz_touch_sensor')

        self.__touch_sensor_left.enable(SAMPLING_INTERVAL)
        self.__touch_sensor_right.enable(SAMPLING_INTERVAL)

        self.__node = rclpy.node.Node('webots_resistance_meter_driver')
        self.__publisher = self.__node.create_publisher(Int32, '/resistance_meter', 1)

        self.__last_measurement_time = 0

    def step(self):

        if self.__supervisor.getTime() - self.__last_measurement_time < SAMPLING_INTERVAL:
            return
        else:
            self.__last_measurement_time = self.__supervisor.getTime()

        print("left", self.__touch_sensor_left.getValues())
        print("right", self.__touch_sensor_right.getValues())

        # self.__publisher.publish(Int8(data=69))
        rclpy.spin_once(self.__node, timeout_sec=0)
