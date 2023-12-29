cmake_minimum_required(VERSION 3.9)
project(jaka_kinematics)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

add_executable(jaka_kinematics_test src/main.cpp src/jaka_kinematics.cpp) 
