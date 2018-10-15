cmake_minimum_required(VERSION 3.9)  # TODO: determine true minimum required version here

# TODO: determine where project should be defined
project(serialPort VERSION 1.0
				   DESCRIPTION "a simple class to handle serial operations"
				   LANGUAGES CXX)

#
# add a library for the serial port handling
#
add_library(seriallib STATIC serial_port.cpp include/serial_port.h)
target_include_directories(seriallib PUBLIC include)
target_compile_features(seriallib PUBLIC cxx_std_11)
# TODO: want to then install the library to ublox-logger/lib/