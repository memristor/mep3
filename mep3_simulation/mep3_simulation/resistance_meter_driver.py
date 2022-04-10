from functools import reduce
from math import copysign, degrees
from random import randrange

from mep3_msgs.action import ResistanceMeter
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

FORCE_TRESHOLD = 0.01

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
    'y_end': -0.83,
    'yaw_tolerance': 4.6
}

MEASUREMENT_NOISE = 3.0  # percent
MEASUREMENT_INACCURACY = 5.0  # percent


def value_in_range(value, left, right):
    if left > right:
        left, right = right, left
    return value >= left and value <= right


def vector_to_average_scalar(vector):
    return reduce(lambda e, s: e + s, vector) / len(vector)


def axangle_to_yaw(axis_angle):
    assert len(axis_angle) == 4
    x, y, z, theta = axis_angle

    # Assertion is disabled because rotation 0 0 1 0
    # is represented as 0 1 0 0 in Webots

    # Axis vector is approximately z-axis unit vector
    # assert abs(x) < 0.1
    # assert abs(y) < 0.1
    # assert value_in_range(abs(z), 0.9, 1.1)

    return theta * copysign(1.0, z)  # radians


class ResistanceMeterDriver:

    def init(self, webots_node, properties):
        namespace = properties['namespace']
        self.measuring_side = properties['measuringSide']

        assert self.measuring_side in ['left', 'right']

        self.__supervisor = webots_node.robot
        self.__robot = self.__supervisor.getSelf()

        self.__touch_sensor_front = self.__supervisor.getDevice(
            f'hand_{self.measuring_side}_Dz_touch_sensor_front'
        )
        self.__touch_sensor_back = self.__supervisor.getDevice(
            f'hand_{self.measuring_side}_Dz_touch_sensor_back'
        )

        timestep = int(self.__supervisor.getBasicTimeStep())

        self.__touch_sensor_front.enable(timestep)
        self.__touch_sensor_back.enable(timestep)

        self.__arm = self.__supervisor.getDevice(
            f'hand_{self.measuring_side}_Dz'
        )

        self.__node = rclpy.node.Node(f'webots_resistance_meter_{self.measuring_side}_driver')
        self.__action = ActionServer(
            self.__node,
            ResistanceMeter,
            f'{namespace}/resistance_meter/{self.measuring_side}',
            execute_callback=self.__execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.__goal_callback,
            cancel_callback=self.__cancel_callback
        )

    def __goal_callback(self, _):
        return GoalResponse.ACCEPT

    def __cancel_callback(self, _):
        return CancelResponse.ACCEPT

    def __execute_callback(self, goal_handle):
        result = ResistanceMeter.Result()
        resistance = 0

        if goal_handle.is_cancel_requested:
            goal_handle.cancelled()

        position = self.discretize_robot_position()
        force = self.measure_touch_force()

        if position is not None:
            if force >= FORCE_TRESHOLD:
                resistance = EXCAVATION_SQUARES['resistances'][position]
                noise = randrange(
                    resistance * MEASUREMENT_NOISE * -1,
                    resistance * MEASUREMENT_NOISE
                ) / 100.0
                inaccuracy = randrange(
                    resistance * MEASUREMENT_INACCURACY * -1,
                    resistance * MEASUREMENT_INACCURACY
                ) / 100.0
                resistance = int(resistance + noise + inaccuracy)

        result.resistance = resistance
        goal_handle.succeed()

        return result

    def measure_touch_force(self):

        fork_front = self.__touch_sensor_front.getValues()
        fork_back = self.__touch_sensor_back.getValues()

        return vector_to_average_scalar(fork_front) + \
            vector_to_average_scalar(fork_back)

    def discretize_robot_position(self):

        trn = self.__robot.getField('translation').getSFVec3f()
        rot = self.__robot.getField('rotation').getSFRotation()

        if value_in_range(
            trn[1],
            EXCAVATION_SQUARES['y_start'], EXCAVATION_SQUARES['y_end']
        ):

            # Robot's yaw in degrees
            yaw = degrees(axangle_to_yaw(rot))

            # Robot's orientation
            o = None
            if value_in_range(
                yaw,
                0 - EXCAVATION_SQUARES['yaw_tolerance'],
                0 + EXCAVATION_SQUARES['yaw_tolerance']
            ):
                o = 'right'
            elif value_in_range(
                abs(yaw),
                180 - EXCAVATION_SQUARES['yaw_tolerance'],
                180 + EXCAVATION_SQUARES['yaw_tolerance']
            ):
                o = 'left'
            else:
                # Outside yaw range
                return None

            # Robot's Џ hand x-axis center in meters
            x = trn[0] + EXCAVATION_SQUARES[f'x_correction_{o}']
            for i, c in enumerate(EXCAVATION_SQUARES['x_center']):
                # Inside x-axis center range
                if value_in_range(
                    x,
                    c - EXCAVATION_SQUARES['x_tolerance'],
                    c + EXCAVATION_SQUARES['x_tolerance']
                ):
                    return i

            # Outside x-axis center ranges
            return None

        else:

            # Outside y-axis range
            return None

    def step(self):

        rclpy.spin_once(self.__node, timeout_sec=0)
