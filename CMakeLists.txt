cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE "DEBUG")
project( test )
find_package( OpenCV REQUIRED )
add_executable( test 
main.cc 
ORBextractor.cc
ORBmatcher.cc
Frame.cc)
target_link_libraries( test ${OpenCV_LIBS} )
