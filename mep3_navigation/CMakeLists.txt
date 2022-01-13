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
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mep3_msgs REQUIRED)
find_package(ruckig REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(distance_angle_regulator
  src/distance_angle/distance_angle_regulator.cpp
  src/distance_angle/pid_regulator.c
  src/distance_angle/motion_profile.cpp
)

target_link_libraries(distance_angle_regulator ruckig::ruckig)

ament_target_dependencies(distance_angle_regulator
  rclcpp
  nav_msgs
  nav2_msgs
  nav2_util
  geometry_msgs
  mep3_msgs
  ruckig
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

install(TARGETS
distance_angle_regulator
DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # TODO: Delete the following lines.
  # The instructions ignore uncristify tests.
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
