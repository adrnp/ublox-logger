#pragma once

#include <stdint.h>
#include <cstring>

struct satellite_info_s {
	uint64_t timestamp; // required for logger
	uint8_t count;
	uint8_t svid[20];
	uint8_t used[20];
	uint8_t elevation[20];
	uint8_t azimuth[20];
	uint8_t snr[20];
	uint8_t _padding0[3]; // required for logger

	static const uint8_t SAT_INFO_MAX_SATELLITES = 20;
};
