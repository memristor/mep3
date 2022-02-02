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

positions = [(-1.34, 0.437, 10), (-1.12, 0.706, 10), (-0.761, 0.786, 20),
             (-0.13, 0.792, 10), (-0.764, 0.53, 10), (-0.848, 0.297, 15),
             (-0.713, 0.063, 10), (-0.603, -0.127, 15)]


def destination_achieved(curr, dest, epsilon):
    return (abs(curr[0] - dest[0]) < epsilon
            and abs(curr[1] - dest[1]) < epsilon)


def time_achieved(time_period):
    ret_value = False
    t = supervisor.getTime()
    while supervisor.getTime() - t < time_period:
        supervisor.step(timestep)
        ret_value = False
    else:
        ret_value = True

    return ret_value


delta = 0.001
epsilon = 0.05
achieved_destination = False
destination = positions[random.randint(0, len(positions) - 1)]
while supervisor.step(timestep) != -1:

    current_position = opponent_field.getSFVec3f()

    if (achieved_destination):
        destination = random.choice(positions)

    if (destination_achieved(current_position, destination, epsilon)):

        if (time_achieved(destination[2])):
            t = supervisor.getTime()

            achieved_destination = True

    else:
        achieved_destination = False
        if current_position[0] < destination[0]:
            current_position[0] += delta
        if current_position[1] < destination[1]:
            current_position[1] += delta
        if current_position[0] > destination[0]:
            current_position[0] -= delta
        if current_position[1] > destination[1]:
            current_position[1] -= delta
        if (current_position[0] != destination[0]
                and current_position[1] != destination[1]):
            opponent_field.setSFVec3f(current_position)

        else:
            current_position[0] = destination[0]
            current_position[1] = destination[1]

    pass

    # Enter here exit cleanup code.
