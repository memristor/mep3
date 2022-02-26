import math
import os
import random


# Otherwise, the controller Python module tries to link a wrong version of the library.
if os.getenv('ROS_DISTRO') is not None:
    from webots_ros2_driver_webots.controller import Supervisor
else:
    from controller import Supervisor

THETA = 0.005
DELTA = 0.001 * random.uniform(1, 2)
POSITIONS_1 = [(-1.0, 0.437, 10), (-1.12, 0.706, 10), (-0.761, 0.786, 20),
               (-0.13, 0.792, 10), (-0.764, 0.53, 10), (-0.848, 0.297, 15),
               (-0.713, 0.063, 10), (-0.603, -0.127, 15)]
POSITIONS_2 = [(-1.0, 0.0238, 10), (-1.22, -0.257, 10), (-1.13, -0.646, 10),
               (-0.738, -0.904, 15), (-0.565, -0.666, 20),
               (-0.827, -0.412, 15)]


def is_destination_achieved(current_position, destination_position, epsilon=0.05):
    return abs(current_position[0] -
               destination_position[0]) < epsilon and abs(
                   current_position[1] - destination_position[1]) < epsilon


def wait_at_destination(supervisor, timestep, time_period):
    current_time = supervisor.getTime()
    while supervisor.getTime() - current_time < time_period:
        supervisor.step(timestep)


def is_not_rotation_achieved(current_rotation_angle, required_angle, epsilon=0.05):
    return abs(current_rotation_angle - required_angle) > epsilon


def set_angle(current_angle, required_angle, theta):
    if current_angle < required_angle:
        current_angle += theta
    else:
        current_angle -= theta
    return current_angle


def target_angle_fun(supervisor, destination_x, destination_y):
    opponent_node = supervisor.getSelf()
    opponent_field = opponent_node.getField('translation')
    current_position = opponent_field.getSFVec3f()
    return math.atan2(destination_y - current_position[1], destination_x - current_position[0])


def main():
    supervisor = Supervisor()

    # get the time step of the current world.
    timestep = int(supervisor.getBasicTimeStep())
    opponent_node = supervisor.getSelf()
    opponent_field = opponent_node.getField('translation')
    opponent_rotation_field = opponent_node.getField('rotation')

    positions = POSITIONS_1 if supervisor.getName() == 'opponent_box_big' else POSITIONS_2

    destination = positions[random.randint(0, len(positions) - 1)]

    while supervisor.step(timestep) != -1:
        current_position = opponent_field.getSFVec3f()
        current_rotation_angle = opponent_rotation_field.getSFRotation()

        if is_destination_achieved(current_position, destination):
            wait_at_destination(supervisor, timestep, destination[2])
            destination = random.choice(positions)

        else:
            target_angle = target_angle_fun(supervisor, destination[0], destination[1])
            if is_not_rotation_achieved(current_rotation_angle[3], target_angle):
                current_rotation_angle[3] = set_angle(
                    current_rotation_angle[3], target_angle, THETA)

            if is_not_rotation_achieved(current_rotation_angle[3], target_angle):
                opponent_rotation_field.setSFRotation(current_rotation_angle)

            else:

                if current_position[0] < destination[0]:
                    current_position[0] += DELTA
                else:
                    current_position[0] -= DELTA

                if current_position[1] < destination[1]:
                    current_position[1] += DELTA
                else:
                    current_position[1] -= DELTA

                if current_position[0] != destination[0] and current_position[1] != destination[1]:
                    opponent_field.setSFVec3f(current_position)

                else:
                    current_position[0] = destination[0]
                    current_position[1] = destination[1]


if __name__ == '__main__':
    main()
