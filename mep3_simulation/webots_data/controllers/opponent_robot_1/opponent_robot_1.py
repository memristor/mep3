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

positions = [(-1.4, 0.0238, 10), (-1.22, -0.257, 10), (-1.13, -0.646, 10),
             (-0.738, -0.904, 15), (-0.565, -0.666, 20), (-0.827, -0.412, 15)]


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
