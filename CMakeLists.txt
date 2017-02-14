cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE "DEBUG")
project( test )

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O0 -march=native -std=c++0x")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O0 -march=native -std=c++0x")

find_package( OpenCV REQUIRED )
add_executable( test 
main.cc 
ORBextractor.cc
ORBmatcher.cc
Frame.cc
convert.cc
)
target_link_libraries( test ${OpenCV_LIBS} )
