cmake_minimum_required(VERSION 2.8)
project("Driver Test")

# we want to compile for c++11
add_definitions(-std=c++11)

# include the GPS Drivers
include_directories("${PROJECT_SOURCE_DIR}/GpsDrivers")

# get all the files to compile
file(GLOB SOURCES "*.cpp")
file(GLOB DRIVER_SOURCES "GpsDrivers/src/*.cpp")



# make the executable
add_executable(drivers	${SOURCES} ${DRIVER_SOURCES})
