cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_walker)

## For compiling c++11 functions
add_compile_options(-std=c++11)

## Find catkin macros and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs move_base_msgs sensor_msgs)

## Declare catkin package 
catkin_package()

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable(walker src/main.cpp src/walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})
