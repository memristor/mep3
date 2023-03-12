cmake_minimum_required(VERSION 3.5)
project(mep3_navigation)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mep3_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ruckig REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(laser_filters REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(can_msgs REQUIRED)

include_directories(include)


# RPP controller
add_library(regulated_pure_pursuit_controller SHARED
        src/regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.cpp)
target_link_libraries(regulated_pure_pursuit_controller ruckig::ruckig)
target_compile_definitions(regulated_pure_pursuit_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(regulated_pure_pursuit_controller
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)

# temporal_obstacle_layer
add_library(
  temporal_obstacle_layer
  SHARED
  src/temporal_obstacle_layer/temporal_obstacle_layer.cpp
)
ament_target_dependencies(
  temporal_obstacle_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  mep3_msgs
  diagnostic_msgs
)
target_include_directories(
  temporal_obstacle_layer
  PRIVATE
  include
)
install(
  TARGETS temporal_obstacle_layer
  DESTINATION lib
)
pluginlib_export_plugin_description_file(nav2_costmap_2d temporal_obstacle_layer.xml)

# dilation_layer
add_library(
  dilation_layer
  SHARED
  src/dilation_layer/dilation_layer.cpp
)
ament_target_dependencies(
  dilation_layer
  rclcpp
  nav2_costmap_2d
  pluginlib
  OpenCV
)
target_include_directories(
  dilation_layer
  PRIVATE
  include
)
install(
  TARGETS dilation_layer
  DESTINATION lib
)
pluginlib_export_plugin_description_file(nav2_costmap_2d dilation_layer.xml)

# move_behavior
add_library(
  move_behavior
  SHARED
  src/move_behavior/move_behavior.cpp
)
ament_target_dependencies(
  move_behavior
  rclcpp
  nav2_costmap_2d
  pluginlib
  rclcpp_action
  mep3_msgs
  can_msgs
)
target_include_directories(
  move_behavior
  PRIVATE
  include
)
install(
  TARGETS move_behavior
  DESTINATION lib
)
target_link_libraries(move_behavior ruckig::ruckig)
pluginlib_export_plugin_description_file(nav2_core move_behavior.xml)

# Assets
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

install(TARGETS
  regulated_pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(regulated_pure_pursuit_controller)

pluginlib_export_plugin_description_file(nav2_core mep3_regulated_pure_pursuit_controller.xml)

ament_package()
