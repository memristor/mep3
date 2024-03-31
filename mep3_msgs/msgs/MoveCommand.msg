uint8 REVERSING_NO = 0
uint8 REVERSING_AUTO = 1
uint8 REVERSING_FORCE = 2

uint8 MODE_ROTATE_TOWARDS_GOAL = 4
uint8 MODE_TRANSLATE = 2
uint8 MODE_ROTATE_AT_GOAL = 1

std_msgs/Header header
string odom_frame

geometry_msgs/Pose2D target

mep3_msgs/MoveProperties linear_properties
mep3_msgs/MoveProperties angular_properties
bool ignore_obstacles
bool enable_sensors
builtin_interfaces/Duration timeout
uint8 reversing
uint8 mode

# TODO
float64 translate_yaw_error_tolerance
