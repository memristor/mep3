from controller import Supervisor

from enum import auto, Enum
import math
import os
import random


THETA = 0.01
RADIUS = 0.3

POSITIONS_1 = [(-0.775, 0.375, 2), (0.045, 0.375, 1), (0.045, -0.725, 2)]

POSITIONS_2 = [(0.535, 0.775, 2), (-0.595, 0.775, 1), (-0.295, -1.235, 2)]


class States(Enum):
    ROTATE = auto()
    GO_TO = auto()


def is_destination_achieved(current_position, destination_position, epsilon=0.05):
    return abs(current_position[0] -
               destination_position[0]) < epsilon and abs(
                   current_position[1] - destination_position[1]) < epsilon


def wait_at_destination(supervisor, timestep, time_period):
    current_time = supervisor.getTime()
    while supervisor.getTime() - current_time < time_period:
        supervisor.step(timestep)


def is_rotation_achieved(current_rotation_angle, required_angle, epsilon=0.02):
    return abs(current_rotation_angle - required_angle) < epsilon


def set_angle(current_angle, required_angle, theta):
    if current_angle < required_angle:
        current_angle += theta
    else:
        current_angle -= theta
    return current_angle


def get_target_angle(supervisor, destination_x, destination_y):
    opponent_node = supervisor.getSelf()
    opponent_field = opponent_node.getField('translation')
    current_position = opponent_field.getSFVec3f()
    return math.atan2(destination_y - current_position[1], destination_x - current_position[0])


def are_colliding(translation, translation_memristor):
    position = translation.getSFVec3f()
    position_memristor = translation_memristor.getSFVec3f()

    dx = position_memristor[0] - position[0]
    dy = position_memristor[1] - position[1]

    return math.sqrt(dx ** 2 + dy ** 2) < RADIUS


def main():
    supervisor = Supervisor()
    # use_opponents = 'MEP3_OPPONENTS' in os.environ and \
        # os.environ['MEP3_OPPONENTS'].lower() == 'true'
    # if not use_opponents:
        # return

    # get the time step of the current world.
    timestep = int(supervisor.getBasicTimeStep())
    opponent_node = supervisor.getSelf()
    opponent_field = opponent_node.getField('translation')
    opponent_rotation_field = opponent_node.getField('rotation')

    memristor_robot = supervisor.getFromDef('ROBOT_OPPONENT_BIG')
    memristor_robot_translation = memristor_robot.getField('translation')

    positions = POSITIONS_1 if supervisor.getName() == 'opponent_box_big' else POSITIONS_2

    destination = positions[random.randint(0, len(positions) - 1)]

    next_state = States.ROTATE

    while supervisor.step(timestep) != -1:
        current_position = opponent_field.getSFVec3f()
        current_rotation_angle = opponent_rotation_field.getSFRotation()

        if next_state == States.ROTATE:

            target_angle = get_target_angle(supervisor, destination[0], destination[1])
            if not is_rotation_achieved(current_rotation_angle[3], target_angle):
                current_rotation_angle[3] = set_angle(
                    current_rotation_angle[3], target_angle, THETA)

            if not is_rotation_achieved(current_rotation_angle[3], target_angle):
                opponent_rotation_field.setSFRotation(current_rotation_angle)

            else:
                next_state = States.GO_TO

        if next_state == States.GO_TO:

            if is_destination_achieved(current_position, destination):
                wait_at_destination(supervisor, timestep, destination[2])
                destination = random.choice(positions)
                next_state = States.ROTATE

            else:
                velocity_factor = random.uniform(0.0009, 0.002)

                delta_x = math.cos(target_angle) * velocity_factor
                delta_y = math.sin(target_angle) * velocity_factor

                if not are_colliding(opponent_field, memristor_robot_translation):

                    current_position[0] += delta_x
                    current_position[1] += delta_y

                if current_position[0] != destination[
                        0] and current_position[1] != destination[1]:
                    opponent_field.setSFVec3f(current_position)

                else:
                    current_position[0] = destination[0]
                    current_position[1] = destination[1]


if __name__ == '__main__':
    main()
