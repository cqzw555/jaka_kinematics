cmake_minimum_required(VERSION 3.9)
project(jaka_kinematics LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
include_directories(/usr/include/python3.6m)
find_package(pybind11 REQUIRED)
# find_package(Python REQUIRED)
# find_package(catkin REQUIRED COMPONENTS roscpp rospy)

# catkin_package(
#   INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp rospy
# )

add_library(jaka_kinematics SHARED src/jaka_kinematics.cpp)
pybind11_add_module(jaka_kinematics_pybind11 src/jaka_kinematics_pybind11.cpp src/jaka_kinematics.cpp)
