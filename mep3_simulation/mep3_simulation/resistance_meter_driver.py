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

        self.__touch_sensor_left_front = self.__supervisor.getDevice('hand_left_Dz_touch_sensor_front')
        self.__touch_sensor_left_back = self.__supervisor.getDevice('hand_left_Dz_touch_sensor_back')
        self.__touch_sensor_right_front = self.__supervisor.getDevice('hand_right_Dz_touch_sensor_front')
        self.__touch_sensor_right_back = self.__supervisor.getDevice('hand_right_Dz_touch_sensor_back')

        self.__touch_sensor_left_front.enable(int(SAMPLING_INTERVAL * 10))
        self.__touch_sensor_left_back.enable(int(SAMPLING_INTERVAL * 10))
        self.__touch_sensor_right_front.enable(int(SAMPLING_INTERVAL * 10))
        self.__touch_sensor_right_back.enable(int(SAMPLING_INTERVAL * 10))

        self.__node = rclpy.node.Node('webots_resistance_meter_driver')
        self.__publisher = self.__node.create_publisher(Int32, '/resistance_meter', 1)

        self.__last_measurement_time = 0

    def vector_to_average_scalar(vector):

        return reduce(lambda e, s: e + s, vector) / len(vector)

    def measure_touch_force(self):

        left_front = self.__touch_sensor_left_front.getValues()
        left_back = self.__touch_sensor_left_back.getValues()
        right_front = self.__touch_sensor_right_front.getValues()
        right_back = self.__touch_sensor_right_back.getValues()

        left = ResistanceMeterDriver.vector_to_average_scalar(left_front) + \
                 ResistanceMeterDriver.vector_to_average_scalar(left_back)
        right = ResistanceMeterDriver.vector_to_average_scalar(right_front) + \
                 ResistanceMeterDriver.vector_to_average_scalar(right_back)

        return left, right

    def step(self):

        if self.__supervisor.getTime() - self.__last_measurement_time < SAMPLING_INTERVAL:
            return

        force = self.measure_touch_force()
        self.__last_measurement_time = self.__supervisor.getTime()

        # self.__publisher.publish(Int8(data=69))
        rclpy.spin_once(self.__node, timeout_sec=0)
