import random

from controller import Supervisor
# create the Robot instance.

supervisor = Supervisor()

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())
opponent_node = supervisor.getSelf()
name_field = opponent_node.getField('name')
opponent_field = opponent_node.getField('translation')
name = name_field.getSFString()

position = [0, 0, 0]

POSITIONS_1 = [(-1.34, 0.437, 10), (-1.12, 0.706, 10), (-0.761, 0.786, 20),
               (-0.13, 0.792, 10), (-0.764, 0.53, 10), (-0.848, 0.297, 15),
               (-0.713, 0.063, 10), (-0.603, -0.127, 15)]

POSITIONS_2 = [(-1.4, 0.0238, 10), (-1.22, -0.257, 10), (-1.13, -0.646, 10),
               (-0.738, -0.904, 15), (-0.565, -0.666, 20),
               (-0.827, -0.412, 15)]


def destination_achieved(current_position, destination_position, epsilon=0.05):
    return abs(current_position[0] -
               destination_position[0]) < epsilon and abs(
                   current_position[1] - destination_position[1]) < epsilon


def is_time_achieved(time_period):
    ret_value = False
    current_time = supervisor.getTime()

    while supervisor.getTime() - current_time < time_period:
        supervisor.step(timestep)
    else:
        ret_value = True

    return ret_value


DELTA = 0.001
achieved_destination = False

positions = POSITIONS_1 if supervisor.getName() == 'opponent_box' else POSITIONS_2

destination = positions[random.randint(0, len(positions) - 1)]

while supervisor.step(timestep) != -1:

    current_position = opponent_field.getSFVec3f()

    if achieved_destination:
        destination = random.choice(positions)

    if (destination_achieved(current_position, destination)):

        if is_time_achieved(destination[2]):
            achieved_destination = True

    else:
        achieved_destination = False
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
