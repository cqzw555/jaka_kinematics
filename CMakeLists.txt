cmake_minimum_required(VERSION 3.9)
project(jaka_kinematics)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy
)

add_library( jaka_kinematics SHARED src/jaka_kinematics.cpp)

add_executable(jaka_kinematics_test src/main.cpp) 
add_executable(forward_test src/forward_test.cpp) 
add_executable(inverse_test src/inverse_test.cpp) 

target_link_libraries(jaka_kinematics_test jaka_kinematics)
target_link_libraries(forward_test jaka_kinematics)
target_link_libraries(inverse_test jaka_kinematics)