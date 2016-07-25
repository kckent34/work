cmake_minimum_required(VERSION 2.8.3)
project(motor_init)
include_directories(include)
file(GLOB SOURCES "src/*.cpp")
add_executable(motor_init ${SOURCES})

