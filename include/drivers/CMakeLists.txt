cmake_minimum_required(VERSION 3.9)  # TODO: determine true minimum required version here

# TODO: determine where project should be defined
project(GpsDrivers VERSION 1.0
				   DESCRIPTION "a submodule containing a set of drivers for a variety of GPS units"
				   LANGUAGES CXX)

#
# add a library for the gps drivers
#
include_directories("${PROJECT_SOURCE_DIR}/GpsDrivers")  # TODO: this should be done differently
file(GLOB DRIVER_SOURCES "GpsDrivers/src/*.cpp")  # TODO: this should be done without GLOB
add_library(gpsdrivers STATIC GpsDrivers/src/ashtech.cpp
							  GpsDrivers/src/gps_helper.cpp
							  GpsDrivers/src/mtk.cpp
							  GpsDrivers/src/rtcm.cpp
							  GpsDrivers/src/ubx.cpp)
target_include_directories(gpsdrivers PUBLIC GpsDrivers/src/)  # NOTE: for this library the headers needed are also in the source file


# TODO: want to then install the library to ublox-logger/lib/