from functools import reduce
from math import pi

from mep3_msgs.action import ResistanceMeter
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from transforms3d.taitbryan import axangle2euler
from std_msgs.msg import Int32

# sudo pip3 install transforms3d

FORCE_TRESHOLD = 1

EXCAVATION_SQUARES = {
    'x_center': [
        -0.8325, -0.6475, -0.4625, -0.2775, -0.0925,
        0.0925, 0.2775, 0.4625, 0.6475, 0.8325
    ],
    'resistances': [
        1000, 1000, 4700, 470, 1000,
        1000, 470, 4700, 470, 470
    ],
    'x_correction_left': 0.0285,
    'x_correction_right': -0.0285,
    'x_tolerance': 0.015,
    'y_start': -0.785,
    'y_end': -0.81,
    'yaw_tolerance': 4.6
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
        except Exception:
            pass

        namespace = properties['namespace']

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

        self.__touch_sensor_left_front.enable(1)
        self.__touch_sensor_left_back.enable(1)
        self.__touch_sensor_right_front.enable(1)
        self.__touch_sensor_right_back.enable(1)

        self.__arm_left = self.__supervisor.getDevice('hand_left_Dz')
        self.__arm_right = self.__supervisor.getDevice('hand_right_Dz')

        self.__node = rclpy.node.Node('webots_resistance_meter_driver')
        self.__action = ActionServer(
            self.__node,
            ResistanceMeter,
            f'{namespace}/resistance_meter',
            execute_callback=self.__execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.__goal_callback,
            cancel_callback=self.__cancel_callback
        )

        self.__last_measurement_time = 0

    def __goal_callback(self, _):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _):
        return CancelResponse.ACCEPT

    def __execute_callback(self, goal_handle):
        result = ResistanceMeter.Result()
        resistance = None

        if goal_handle.is_cancel_requested:
            goal_handle.cancelled()

        measuring_side = goal_handle.request.measuring_side
        position = self.discretize_robot_position()
        force = self.measure_touch_force()

        if position is not None:
            if (measuring_side == 'left' and force[0] >= FORCE_TRESHOLD) or \
                    (measuring_side == 'right' and force[1] >= FORCE_TRESHOLD):
                resistance = EXCAVATION_SQUARES["resistances"][position]

        if resistance is None:
            # No resistance detected while measuring
            result.resistance = 0
            goal_handle.abort()
        else:
            result.resistance = resistance
            goal_handle.succeed()

        return result

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

        trn = self.__robot.getField('translation').getSFVec3f()
        rot = self.__robot.getField('rotation').getSFRotation()

        if value_in_range(trn[1], EXCAVATION_SQUARES['y_start'], EXCAVATION_SQUARES['y_end']):

            # Robot's yaw in degrees
            y = axangle2euler((rot[0], rot[1], rot[2]), rot[3])[0] * 180 / pi

            # Robot's orientation
            o = None
            if value_in_range(y, 0 - EXCAVATION_SQUARES['yaw_tolerance'], 0 + EXCAVATION_SQUARES['yaw_tolerance']):
                o = 'right'
            elif value_in_range(abs(y), 180 - EXCAVATION_SQUARES['yaw_tolerance'], 180 + EXCAVATION_SQUARES['yaw_tolerance']):
                o = 'left'
            else:
                # Outside yaw range
                return None

            # Robot's Џ hand x-axis center in meters
            x = trn[0] + EXCAVATION_SQUARES[f'x_correction_{o}']
            for i, c in enumerate(EXCAVATION_SQUARES['x_center']):
                # Inside x-axis center range
                if value_in_range(x, c - EXCAVATION_SQUARES['x_tolerance'], c + EXCAVATION_SQUARES['x_tolerance']):
                    return i

            # Outside x-axis center ranges
            return None

        else:

            # Outside y-axis range
            return None

    def step(self):

        rclpy.spin_once(self.__node, timeout_sec=0)
