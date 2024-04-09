cmake_minimum_required(VERSION 3.5)
project(mep3_hardware)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(mep3_msgs REQUIRED)
find_package(nav2_util REQUIRED)

# Motion Hardware Interface
add_library(
  mep3_motion_hardware_interface
  SHARED
  src/motion_hardware_interface/motion_hardware_interface.cpp
  src/motion_hardware_interface/motion_board_driver.cpp
)
target_include_directories(
  mep3_motion_hardware_interface
  PRIVATE
  include
)
ament_target_dependencies(
  mep3_motion_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface mep3_motion_hardware_interface.xml)
install(
  TARGETS mep3_motion_hardware_interface
  DESTINATION lib
)

# CAN GPIO Hardware Interface
add_library(
  mep3_can_gpio_hardware_interface
  SHARED
  src/can_gpio_hardware_interface/can_gpio_hardware_interface.cpp
)
target_include_directories(
  mep3_can_gpio_hardware_interface
  PRIVATE
  include
)
ament_target_dependencies(
  mep3_can_gpio_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface mep3_can_gpio_hardware_interface.xml)
install(
  TARGETS mep3_can_gpio_hardware_interface
  DESTINATION lib
)

# Dynamixel Hardware Interface
add_library(
  dynamixel_hardware
  SHARED
  src/dynamixel_hardware_interface/dynamixel_hardware.cpp
)
target_include_directories(
  dynamixel_hardware
  PRIVATE
  include
)
ament_target_dependencies(
  dynamixel_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  dynamixel_workbench_toolbox
)
pluginlib_export_plugin_description_file(hardware_interface dynamixel_hardware.xml)
install(
  TARGETS dynamixel_hardware
  DESTINATION lib
)

# Socket bridge
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
  TARGETS socketcan_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Assets
install(
  DIRECTORY resource launch
  DESTINATION share/${PROJECT_NAME}
)

# Python
install(PROGRAMS
  mep3_hardware/cinch_driver.py
  mep3_hardware/lcd_driver.py
  mep3_hardware/box_driver.py
  mep3_hardware/ir_sensors.py
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
  sensor_msgs
)
ament_package()
