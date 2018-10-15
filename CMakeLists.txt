cmake_minimum_required(VERSION 3.9)

# define the project
# TODO: see what this line should really be
project(ublox-logger)

# run the cmake file from the serial port directory (?)
add_subdirectory(include/serial_port)
add_subdirectory(include/drivers)

add_subdirectory(src/)