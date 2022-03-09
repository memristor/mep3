from functools import reduce
import rclpy
from std_msgs.msg import Int32
from math import pi

from transforms3d.taitbryan import axangle2euler

# sudo pip3 install transforms3d

SAMPLING_INTERVAL = 0.5  # seconds

EXCAVATION_SQUARES = {
    "x_center": [
        -0.8325, -0.6475, -0.4625, -0.2775, -0.0925,
        0.0925, 0.2775, 0.4625, 0.6475, 0.8325
    ],
    "resistances": [
        1000, 1000, 4700, 470, 1000,
        1000, 470, 4700, 470, 470
    ],
    "x_correction_left": 0.0295,
    "x_correction_right": -0.0335,
    "x_tolerance": 0.015,
    "y_start": -0.785,
    "y_end": -0.81,
    "yaw_tolerance": 4.6
}


def value_in_range(value, left, right):
    if left > right:
        left, right = right, left
    return value >= left and value <= right


def vector_to_average_scalar(vector):
    return reduce(lambda e, s: e + s, vector) / len(vector)


class ResistanceMeterDriver:

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:  # noqa: E501
            # logging.exception("WaitMatchStartDriver")
            pass  # noqa: E501

        self.__supervisor = webots_node.robot
        self.__robot = self.__supervisor.getSelf()

        self.__touch_sensor_left_front = self.__supervisor.getDevice(
            'hand_left_Dz_touch_sensor_front'
                )
        self.__touch_sensor_left_back = self.__supervisor.getDevice(
            'hand_left_Dz_touch_sensor_back'
            )
        self.__touch_sensor_right_front = self.__supervisor.getDevice(
            'hand_right_Dz_touch_sensor_front'
            )
        self.__touch_sensor_right_back = self.__supervisor.getDevice(
            'hand_right_Dz_touch_sensor_back'
            )

        self.__touch_sensor_left_front.enable(int(SAMPLING_INTERVAL * 10))
        self.__touch_sensor_left_back.enable(int(SAMPLING_INTERVAL * 10))
        self.__touch_sensor_right_front.enable(int(SAMPLING_INTERVAL * 10))
        self.__touch_sensor_right_back.enable(int(SAMPLING_INTERVAL * 10))

        self.__node = rclpy.node.Node('webots_resistance_meter_driver')
        self.__publisher = self.__node.create_publisher(
            Int32, '/resistance_meter', 1)

        self.__last_measurement_time = 0

    def measure_touch_force(self):

        left_front = self.__touch_sensor_left_front.getValues()
        left_back = self.__touch_sensor_left_back.getValues()
        right_front = self.__touch_sensor_right_front.getValues()
        right_back = self.__touch_sensor_right_back.getValues()

        left = vector_to_average_scalar(left_front) + \
            vector_to_average_scalar(left_back)
        right = vector_to_average_scalar(right_front) + \
            vector_to_average_scalar(right_back)

        return left, right

    def discretize_robot_position(self):

        trn = self.__robot.getField("translation").getSFVec3f()
        rot = self.__robot.getField("rotation").getSFRotation()

        if ResistanceMeterDriver.value_in_range(trn[1], EXCAVATION_SQUARES["y_start"], EXCAVATION_SQUARES["y_end"]):

            # Robot's yaw in degrees
            y = axangle2euler((rot[0], rot[1], rot[2]), rot[3])[0] * 180 / pi

            # Robot's orientation
            o = None
            if value_in_range(y, 0 - EXCAVATION_SQUARES["yaw_tolerance"], 0 + EXCAVATION_SQUARES["yaw_tolerance"]):
                o = "right"
            elif value_in_range(abs(y), 180 - EXCAVATION_SQUARES["yaw_tolerance"], 180 + EXCAVATION_SQUARES["yaw_tolerance"]):
                o = "left"
            else:
                # Outside yaw range
                return None

            # Robot's Џ hand x-axis center in meters
            x = trn[0] + EXCAVATION_SQUARES[f"x_correction_{o}"]
            for i, c in enumerate(EXCAVATION_SQUARES["x_center"]):
                # Inside x-axis center range
                if value_in_range(x, c - EXCAVATION_SQUARES["x_tolerance"], c + EXCAVATION_SQUARES["x_tolerance"]):
                    return i

            # Outside x-axis center ranges
            return None

        else:

            # Outside y-axis range
            return None

    def step(self):

        if self.__supervisor.getTime() - self.__last_measurement_time < SAMPLING_INTERVAL:
            return

        force = self.measure_touch_force()
        print(self.discretize_robot_position())

        self.__last_measurement_time = self.__supervisor.getTime()

        # self.__publisher.publish(Int8(data=69))
        rclpy.spin_once(self.__node, timeout_sec=0)
