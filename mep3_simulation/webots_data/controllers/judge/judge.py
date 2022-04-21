"""Initializes the match."""

from math import pi
import os

# Otherwise, the controller Python module tries to link a wrong version of the library.
if os.getenv('ROS_DISTRO') is not None:
    from webots_ros2_driver_webots.controller import Supervisor
else:
    from controller import Supervisor


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

    # Statuette and replica
    statuette = ObjectManipulator(supervisor, 'STATUETTE')
    replica = ObjectManipulator(supervisor, 'REPLICA')

    # Set initial poses
    if color == 'yellow':
        robot_big.set_position(x=-1.2491, y=0.1, theta=-pi / 2)
        robot_small.set_position(x=-1.21, y=0.463, theta=0)
        robot_opponent_big.set_position(x=1.26, y=0.46, theta=pi)
        robot_opponent_small.set_position(x=1.26, y=0.128, theta=pi)

        supervisor.step(timestep)

        statuette.set_position(x=-1.267, y=-0.768, z=0.125, theta=0.785)
        replica.set_position(x=-1.2621, y=0.17, z=0.1579, theta=pi)
    else:
        robot_big.set_position(x=1.2491, y=0.1, theta=-pi / 2)
        robot_small.set_position(x=1.21, y=0.463, theta=pi)
        robot_opponent_big.set_position(x=-1.26, y=0.46, theta=0)
        robot_opponent_small.set_position(x=-1.26, y=0.128, theta=0)

        supervisor.step(timestep)

        statuette.set_position(x=1.267, y=-0.768, z=0.125, theta=2.356)
        replica.set_position(x=1.2491, y=0.1521, z=0.1579, theta=pi / 2)

    # Do something
    while supervisor.step(timestep) != -1:
        break


if __name__ == '__main__':
    main()
