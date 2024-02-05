cmake_minimum_required(VERSION 3.0.2)
project(static_transform)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_executable(static_tf src/static_tf.cpp)
target_link_libraries(static_tf  ${catkin_LIBRARIES} )

