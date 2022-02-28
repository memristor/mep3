cmake_minimum_required(VERSION 3.5)
project(mep3_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(can_msgs REQUIRED)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/robot_hardware_interface.cpp
  src/motion_board_driver.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface mep3_driver_ros2_control.xml)

add_executable(socketcan_bridge src/socketcan_bridge.cpp)
target_include_directories(
  socketcan_bridge
  PRIVATE
  include
)

ament_target_dependencies(
  socketcan_bridge
  rclcpp
  can_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  TARGETS socketcan_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY resource launch
  DESTINATION share/${PROJECT_NAME}
)

# Python
install(PROGRAMS
  mep3_driver/dynamixel_driver.py
  mep3_driver/cinch_driver.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${PROJECT_NAME})

# Export
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  can_msgs
)
ament_package()
