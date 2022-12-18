"""Initializes the match."""

from math import pi
import os
from controller import Supervisor


INITIAL_POSE_MATRIX = [
    ('big', 'purple', [0.102, 1.0, pi/2]),
    ('small', 'purple', [0.443, 1.0, pi]),
    ('big', 'yellow', [0.102, -1.0, pi/2]),
    ('small', 'yellow', [0.443, -1.0, 0]),
]


class ObjectManipulator:

    def __init__(self, supervisor, def_value):
        self.__node = supervisor.getFromDef(def_value)
        if self.__node is None:
            print(f'ERROR: Node DEF {def_value} cannot be found.')
            return
        self.__translation_field = self.__node.getField('translation')
        self.__rotation_field = self.__node.getField('rotation')

    def set_position(self, *, x=0, y=0, z=0, theta=0):
        if self.__node is None:
            return
        self.__translation_field.setSFVec3f([x, y, z])
        self.__rotation_field.setSFRotation([0.0, 0.0, 1.0, theta])


def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    # Determine the color side
    color = 'purple'
    if 'MEP3_COLOR' in os.environ:
        color = os.environ['MEP3_COLOR']

    # Robots
    robot_big = ObjectManipulator(supervisor, 'ROBOT_BIG')
    robot_small = ObjectManipulator(supervisor, 'ROBOT_SMALL')
    robot_opponent_big = ObjectManipulator(supervisor, 'ROBOT_OPPONENT_BIG')
    robot_opponent_small = ObjectManipulator(supervisor,
                                             'ROBOT_OPPONENT_SMALL')

    # Set initial poses
    pose_big = next(pose[2] for pose in INITIAL_POSE_MATRIX if pose[0] == 'big' and pose[1] == color)
    robot_big.set_position(x=pose_big[0], y=pose_big[1], theta=pose_big[2])

    pose_small = next(pose[2] for pose in INITIAL_POSE_MATRIX if pose[0] == 'small' and pose[1] == color)
    robot_small.set_position(x=pose_small[0], y=pose_small[1], theta=pose_small[2])

    robot_opponent_big.set_position(x=pose_big[0], y=-pose_big[1], theta=pose_big[2])
    robot_opponent_small.set_position(x=pose_small[0], y=-pose_small[1], theta=pose_small[2])

    supervisor.step(timestep)

    # Do something
    while supervisor.step(timestep) != -1:
        break


if __name__ == '__main__':
    main()
