#pragma once

#include <stdint.h>
#include <cstring>

struct vehicle_gps_position_s {
	uint64_t timestamp;
	uint64_t time_utc_usec;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	int32_t alt_ellipsoid;
	float s_variance_m_s;
	float c_variance_rad;
	float eph;
	float epv;
	float hdop;
	float vdop;
	int32_t noise_per_ms;
	int32_t jamming_indicator;
	float vel_m_s;
	float vel_n_m_s;
	float vel_e_m_s;
	float vel_d_m_s;
	float cog_rad;
	int32_t timestamp_time_relative;
	float heading;
	uint8_t fix_type;
	bool vel_ned_valid;
	uint8_t satellites_used;
	uint8_t _padding0[1]; // required for logger
};
