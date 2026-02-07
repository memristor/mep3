cmake_minimum_required(VERSION 3.5)
project(mep3_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/JointPositionCommand.action"
  "msgs/Scoreboard.msg"
  "action/VacuumPumpCommand.action"
  "msgs/TemporalObstacle.msg"
  "msgs/MoveProperties.msg"
  "msgs/MoveCommand.msg"
  "msgs/MoveState.msg"
  "action/Move.action"
  "action/Aruco.action"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
