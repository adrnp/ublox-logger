cmake_minimum_required(VERSION 3.9)

# define the project
# TODO: see what this line should really be
project(ublox-logger)

# set some variables that define install locations
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin/")

# run the cmake file from the sub elements needed
add_subdirectory(include/serial_port)
add_subdirectory(include/drivers)

# run the cmake for the core app
add_subdirectory(src/)