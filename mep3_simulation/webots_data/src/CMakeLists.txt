cmake_minimum_required(VERSION 2.8)
project( proj )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( compile_test compile_test.cpp )
target_link_libraries( compile_test ${OpenCV_LIBS} )

add_executable( aruco aruco.cpp )
target_link_libraries( aruco ${OpenCV_LIBS} )
