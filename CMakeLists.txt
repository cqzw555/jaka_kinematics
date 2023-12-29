cmake_minimum_required(VERSION 3.9)
project(jaka_kinematics)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy
)

add_executable(jaka_kinematics_test src/main.cpp src/jaka_kinematics.cpp) 

add_library( jaka_kinematics SHARED src/jaka_kinematics.cpp)
