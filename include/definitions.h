/**
 * @file definitions.h
 * common platform-specific definitions & abstractions for gps
 */

#pragma once

// TODO: figure out how to do these properly
#define GPS_INFO(...) printf(__VA_ARGS__) //PX4_INFO(__VA_ARGS__)
#define GPS_WARN(...) printf(__VA_ARGS__) //PX4_WARN(__VA_ARGS__)
#define GPS_ERR(...) printf(__VA_ARGS__) //PX4_ERR(__VA_ARGS__)

#include "vehicle_gps_position.h"
#include "satellite_info.h"

#include <unistd.h> //this is POSIX, used for usleep
#include <time.h>
#include <chrono>
#include <iostream>	/* output and printing library */

#define M_PI_F			3.14159265358979323846f
#define M_DEG_TO_RAD_F 		0.01745329251994f
#define M_RAD_TO_DEG_F 		57.2957795130823f
#define M_RAD_TO_DEG 		57.295779513082323

#define gps_usleep usleep

using time_stamp = std::chrono::time_point<std::chrono::system_clock, std::chrono::microseconds>;

typedef uint64_t gps_abstime;

/**
 * Get the current time in us. Function signature:
 * uint64_t hrt_absolute_time()
 */
static inline gps_abstime gps_absolute_time() {
	time_stamp ts = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
	return ts.time_since_epoch().count();
}