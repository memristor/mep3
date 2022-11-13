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

# Joint
add_library(
  joint_controller
  SHARED
  src/joint_controller.cpp
)
ament_target_dependencies(
  joint_controller
  rclcpp
  controller_interface
  pluginlib
  hardware_interface
  rclcpp_lifecycle
  rcpputils
  realtime_tools
)
target_include_directories(
  joint_controller
  PRIVATE
  include
)
install(
  TARGETS joint_controller
  DESTINATION lib
)
pluginlib_export_plugin_description_file(controller_interface joint_controller_plugin.xml)


ament_export_include_directories(
  include
)
ament_export_libraries(joint_controller)
ament_package()
