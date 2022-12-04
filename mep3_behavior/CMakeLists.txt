cmake_minimum_required(VERSION 3.5)
project(mep3_behavior)

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
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(mep3_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Use static linking for BT Nodes
add_compile_definitions(MANUAL_STATIC_LINKING)

# Set path to assets
get_filename_component(ASSETS_DIRECTORY strategies ABSOLUTE [CACHE])
add_compile_definitions(ASSETS_DIRECTORY="${ASSETS_DIRECTORY}")

add_executable(${PROJECT_NAME} src/mep3_behavior_tree.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE USE_BTCPP3_OLD_NAMES=ON)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
  BT::behaviortree_cpp
)
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_action
  mep3_msgs
  nav2_msgs
  can_msgs
  diagnostic_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
