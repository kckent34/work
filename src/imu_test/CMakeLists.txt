cmake_minimum_required(VERSION 2.8.3)
project(imu_test)
add_definitions(-std=c++11)
include_directories(include)
file(GLOB SOURCES "src/*.cpp")
add_executable(imu_test ${SOURCES})

