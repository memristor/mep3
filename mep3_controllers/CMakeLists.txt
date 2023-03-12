cmake_minimum_required(VERSION 3.8)
project(mep3_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcpputils REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(nav2_util REQUIRED)
find_package(mep3_msgs REQUIRED)

# Joint Position Controller
add_library(
  joint_position_controller
  SHARED
  src/joint_position_controller.cpp
)
ament_target_dependencies(
  joint_position_controller
  rclcpp
  controller_interface
  pluginlib
  hardware_interface
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  nav2_util
  mep3_msgs
)
target_include_directories(
  joint_position_controller
  PRIVATE
  include
)
install(
  TARGETS joint_position_controller
  DESTINATION lib
)
pluginlib_export_plugin_description_file(controller_interface joint_position_controller_plugin.xml)

# Pump Contorller
add_library(
  pump_controller
  SHARED
  src/pump_controller.cpp
)
ament_target_dependencies(
  pump_controller
  rclcpp
  controller_interface
  pluginlib
  hardware_interface
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  nav2_util
  mep3_msgs
)
target_include_directories(
  pump_controller
  PRIVATE
  include
)
install(
  TARGETS pump_controller
  DESTINATION lib
)
pluginlib_export_plugin_description_file(controller_interface pump_controller_plugin.xml)


ament_export_include_directories(
  include
)
ament_export_libraries(joint_position_controller)
ament_package()
