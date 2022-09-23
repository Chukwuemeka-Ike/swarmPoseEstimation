cmake_minimum_required(VERSION 3.0.2)
project(swarm_pose_estimator)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
   CATKIN_DEPENDS
   DEPENDS system_lib
)

add_executable( centroid_estimator centroid_estimator.cpp )
target_link_libraries( centroid_estimator PUBLIC ${catkin_LIBRARIES} )