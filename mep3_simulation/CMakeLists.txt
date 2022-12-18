cmake_minimum_required(VERSION 3.8)
project(mep3_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(webots_ros2_control REQUIRED)

# mep3_webots_hardware_interface
add_library(
  mep3_webots_hardware_interface
  SHARED
  src/mep3_webots_hardware_interface.cpp
)
ament_target_dependencies(
  mep3_webots_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  webots_ros2_control
)
target_include_directories(
  mep3_webots_hardware_interface
  PRIVATE
  include
)
install(
  TARGETS mep3_webots_hardware_interface
  DESTINATION lib
)
pluginlib_export_plugin_description_file(webots_ros2_control mep3_webots_hardware_interface.xml)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${PROJECT_NAME})

# resources
install(
  DIRECTORY launch resource webots_data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
