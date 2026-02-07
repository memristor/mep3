cmake_minimum_required(VERSION 3.8)
project(mep3_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(aruco_vision src/camera_aruco.cpp)
ament_target_dependencies(aruco_vision rclcpp std_msgs)

install(TARGETS
  aruco_vision
  DESTINATION lib/${PROJECT_NAME})

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Aruco.action"
)

ament_package()
