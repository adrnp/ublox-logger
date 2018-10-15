cmake_minimum_required(VERSION 3.9)  # TODO: determine proper minimum version

# TODO: figure out the proper project definition
project(logger-core)

# we want to compile for c++11
add_definitions(-std=c++11)

# include some of the project headers
include_directories(../include/)

# make the executable
add_executable(ublox_logger	ublox_logger_main.cpp)
target_link_libraries(ublox_logger gpsdrivers)  # link to the drivers
target_link_libraries(ublox_logger seriallib)   # link to the serial library
