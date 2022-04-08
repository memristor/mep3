from math import pi, inf
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range

class WebotsBinaryRangerDriver:
    """
    Binary ranger returns -inf if object is detected else inf.

    On our simulated robot, the distance sensors return the true distance from
    objects, whereas in the driver we convert them in binary rangers, in order
    to be compliant with the physical system.
    In Webots we have four distance sensors:
    - distance_sensor_front_left
    - distance_sensor_front_right
    - distance_sensor_rear_left
    - distance_sensor_rear_right
    In ros2, we create four topics:
    - binary_ranger_command/front_left
    - binary_ranger_command/front_right
    - binary_ranger_command/rear_left
    - binary_ranger_command/rear_right
    """

    def init(self, webots_node, properties):
        try:
            rclpy.init(args=None)
        except Exception:  # noqa: E501
            pass  # noqa: E501
        self.__executor = MultiThreadedExecutor()
        
        namespace = properties['namespace']
        ranger_position = properties['binaryRangerPosition']

        self.__node = rclpy.create_node(
            f'webots_binary_ranger_{ranger_position}')
        self.__robot = webots_node.robot
        self.__distance_sensor = self.__robot.getDevice(
            f'distance_sensor_{ranger_position}')
        timestep = int(self.__robot.getBasicTimeStep())
        self.__distance_sensor.enable(timestep)
        
        self.__DISTANCE_THRESHOLD_M = 0.3
        self.__FOW_RAD = pi / 6
        timer_period = 0.5  # seconds
            
        self.__publisher = self.create_publisher(Range, f'{namespace}/binary_ranger_command/{ranger_position}', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
            
    def send_goal(self, range):
        # This is a fixed distance ranger, we can write
        # min_range===max_range===distance

        goal_msg = Range.Goal()
        goal_msg.ratiation_type = 1  # ULTRASOUND=0 | INFRARED=1
        goal_msg.field_of_view = self.__FOW_RAD  # radians
        goal_msg.min_range = self.__DISTANCE_THRESHOLD_M  # meters
        goal_msg.max_range = self.__DISTANCE_THRESHOLD_M  # meters
        goal_msg.range = range  # meters

        return self.__publisher.publish(goal_msg)
        
    def timer_callback(self):
        distance = self.__distance_sensor.getValue()
        distance = distance <= self.__DISTANCE_TRESHOLD_M
        if distance:
            self.send_goal(-inf)
        else:
            self.send_goal(inf)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0, executor=self.__executor)
