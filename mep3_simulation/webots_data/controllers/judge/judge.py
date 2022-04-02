"""Initializes the match."""

import os
from math import pi

# Otherwise, the controller Python module tries to link a wrong version of the library.
if os.getenv('ROS_DISTRO') is not None:
    from webots_ros2_driver_webots.controller import Supervisor
else:
    from controller import Supervisor


class RobotManipulator:
    def __init__(self, supervisor, def_value):
        self.__node = supervisor.getFromDef(def_value)
        self.__translation_field = self.__node.getField('translation')
        self.__rotation_field = self.__node.getField('rotation')

    def set_position(self, pose):
        x = pose[0]
        y = pose[1]
        theta = pose[2]
        self.__translation_field.setSFVec3f([x, y, 0.0])
        self.__rotation_field.setSFRotation([0.0, 0.0, 1.0, theta])


def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())

    # Determine the color side
    color = 'purple'
    if 'MEP3_COLOR' in os.environ:
        color = os.environ['MEP3_COLOR']

    # Set initial poses
    if color == 'yellow':
        RobotManipulator(supervisor, 'ROBOT_BIG').set_position([-1.21, 0.17, 0.0])
        RobotManipulator(supervisor, 'ROBOT_OPPONENT_BIG').set_position([1.26, 0.46, pi])
        RobotManipulator(supervisor, 'ROBOT_OPPONENT_SMALL').set_position([1.26, 0.128, pi])
    else:
        RobotManipulator(supervisor, 'ROBOT_BIG').set_position([1.21, 0.17, pi])
        RobotManipulator(supervisor, 'ROBOT_OPPONENT_BIG').set_position([-1.26, 0.46, 0.0])
        RobotManipulator(supervisor, 'ROBOT_OPPONENT_SMALL').set_position([-1.26, 0.128, 0.0])

    # Do something
    while supervisor.step(timestep) != -1:
        break


if __name__ == '__main__':
    main()
