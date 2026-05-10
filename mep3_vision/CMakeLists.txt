cmake_minimum_required(VERSION 3.8)
project(mep3_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(mep3_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)

###################################
# Aruco vision dependencies

add_library(action_server SHARED src/aruco_detection.cpp)

target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(action_server PRIVATE "MEP3_VISION_DLL")

ament_target_dependencies(action_server
  mep3_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  OpenCV
)

rclcpp_components_register_node(action_server
  PLUGIN "mep3_vision::ArucoActionServer"
  EXECUTABLE aruco_action_server
)

install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

###################################
# Camera detection dependencies

add_library(camera_detection_server SHARED
  src/object_detection.cpp
)

target_include_directories(camera_detection_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(camera_detection_server PRIVATE "MEP3_VISION_DLL")

ament_target_dependencies(camera_detection_server
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  mep3_msgs
)

rclcpp_components_register_node(camera_detection_server
  PLUGIN "mep3_vision::ObjectDetection"
  EXECUTABLE object_detection
)

install(TARGETS
  camera_detection_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()

ament_package()