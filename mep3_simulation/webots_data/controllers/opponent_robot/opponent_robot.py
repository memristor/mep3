import os
import random

# Otherwise, the controller Python module tries to link a wrong version of the library.
if os.getenv('ROS_DISTRO') is not None:
    from webots_ros2_driver_webots.controller import Supervisor
else:
    from controller import Supervisor

THETA = 0.01
DELTA = 0.001 * random.uniform(1, 2)
POSITIONS_1 = [(-1.34, 0.437, 1), (-1.12, 0.706, 1), (-0.761, 0.786, 1),
               (-0.13, 0.792, 1), (-0.764, 0.53, 2), (-0.848, 0.297, 1),
               (-0.713, 0.063, 1), (-0.603, -0.127, 1)]
POSITIONS_2 = [(-1.4, 0.0238, 1), (-1.22, -0.257, 1), (-1.13, -0.646, 1),
               (-0.738, -0.904, 1), (-0.565, -0.666, 2),
               (-0.827, -0.412, 1)]


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


def main():
    supervisor = Supervisor()

    # get the time step of the current world.
    timestep = int(supervisor.getBasicTimeStep())
    opponent_node = supervisor.getSelf()
    opponent_field = opponent_node.getField('translation')
    opponent_rotation_field = opponent_node.getField('rotation')

    positions = POSITIONS_1 if supervisor.getName(
    ) == 'opponent_box' else POSITIONS_2

    destination = positions[random.randint(0, len(positions) - 1)]

    while supervisor.step(timestep) != -1:
        current_position = opponent_field.getSFVec3f()
        current_rotation_angle = opponent_rotation_field.getSFRotation()

        if is_destination_achieved(current_position, destination):
            wait_at_destination(supervisor, timestep, destination[2])
            destination = random.choice(positions)
        else:
            if current_position[0] < destination[0]:
                if not is_rotation_achieved(current_rotation_angle[3], -3.14159):
                    if current_rotation_angle[3] < -3.14159:
                        current_rotation_angle[3] += THETA
                    else:
                        current_rotation_angle[3] -= THETA
                opponent_rotation_field.setSFRotation(current_rotation_angle)
                current_position[0] += DELTA

            else:
                if not is_rotation_achieved(current_rotation_angle[3], 0):
                    if current_rotation_angle[3] < 0:
                        current_rotation_angle[3] += THETA
                    else:
                        current_rotation_angle[3] -= THETA
                opponent_rotation_field.setSFRotation(current_rotation_angle)
                current_position[0] -= DELTA

            if current_position[1] < destination[1]:
                if not is_rotation_achieved(current_rotation_angle[3], 1.5708):
                    if current_rotation_angle[3] < 1.5708:
                        current_rotation_angle[3] += THETA
                    else:
                        current_rotation_angle[3] -= THETA
                opponent_rotation_field.setSFRotation(current_rotation_angle)
                current_position[1] += DELTA
            else:
                if not is_rotation_achieved(current_rotation_angle[3],
                                            -1.5708):
                    if current_rotation_angle[3] < -1.5708:
                        current_rotation_angle[3] += THETA
                    else:
                        current_rotation_angle[3] -= THETA
                opponent_rotation_field.setSFRotation(current_rotation_angle)
                current_position[1] -= DELTA

            if current_position[0] != destination[0] and current_position[1] != destination[1]:
                opponent_field.setSFVec3f(current_position)

            else:
                current_position[0] = destination[0]
                current_position[1] = destination[1]


if __name__ == '__main__':
    main()
