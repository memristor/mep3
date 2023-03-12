cmake_minimum_required(VERSION 3.8)
project(mep3_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# resources
install(
  DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

# Python
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${PROJECT_NAME})

ament_package()
