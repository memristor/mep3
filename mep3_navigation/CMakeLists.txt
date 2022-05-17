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

include_directories(include)

# Distance angle regulator
add_executable(distance_angle_regulator
  src/distance_angle/distance_angle_regulator.cpp
  src/distance_angle/pid_regulator.c
  src/distance_angle/motion_profile.cpp
)

target_link_libraries(distance_angle_regulator ruckig::ruckig)

ament_target_dependencies(distance_angle_regulator
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_msgs
  nav2_msgs
  nav2_util
  nav2_core
  nav_2d_utils
  nav_2d_msgs
  geometry_msgs
  mep3_msgs
  std_msgs
  ruckig
  tf2
  tf2_ros
  can_msgs
)
###

# Laser inflator with scan constraining
add_executable(laser_inflator src/laser_inflator/laser_inflator.cpp)
target_include_directories(
  laser_inflator
  PRIVATE
  include
)

ament_target_dependencies(
  laser_inflator
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)
###

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
)

# RPP controller
add_library(regulated_pure_pursuit_controller SHARED
        src/regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.cpp)
target_link_libraries(regulated_pure_pursuit_controller ruckig::ruckig)
# prevent pluginlib from using boost
target_compile_definitions(regulated_pure_pursuit_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(regulated_pure_pursuit_controller
  ${dependencies}
)

# <GlobalUpdatedGoal/> BT condition
add_library(nav2_globally_updated_goal_condition_bt_node SHARED src/plugins/condition/globally_updated_goal_condition.cpp)
ament_target_dependencies(nav2_globally_updated_goal_condition_bt_node ${dependencies})
target_compile_definitions(nav2_globally_updated_goal_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)
install(TARGETS nav2_globally_updated_goal_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Assets
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

install(TARGETS
  distance_angle_regulator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS laser_inflator
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  regulated_pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # TODO: Delete the following lines.
  # The instructions ignore uncristify tests.
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(regulated_pure_pursuit_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core mep3_regulated_pure_pursuit_controller.xml)

ament_package()
