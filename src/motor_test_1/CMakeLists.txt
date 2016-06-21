cmake_minimum_required(VERSION 2.8.3)
project(motor_test_1)
add_definitions(-std=c++11)
include_directories(include)
file(GLOB SOURCES "src/*.cpp")
add_executable(motor_test_1 ${SOURCES})

