#include <cstdlib>	/* standard library (std namespace) */
#include <unistd.h>	/* unix standard library (e.g. read call) */

#include <iostream>	/* output and printing library */
#include <iomanip>	/* std::setfill and setw functions */
#include <fstream>	/* writing to file */

#include <cstdint>	/* for more datatypes */

#include <string.h>	/* use of strings and string related functions */

#include <math.h>	/* for pow and other math functions */


// custom includes
#include "serial_port.h"	/* serial port connection */

// include the messages
#include "vehicle_gps_position.h"

// include the drivers
#include "GpsDrivers/src/ubx.h"		/* u-blox driver */

#define TIMEOUT_5HZ 500

using namespace std;



typedef enum {
	GPS_DRIVER_MODE_NONE = 0,
	GPS_DRIVER_MODE_UBX,
	GPS_DRIVER_MODE_MTK,
	GPS_DRIVER_MODE_ASHTECH
} gps_driver_mode_t;


/* class for dynamic allocation of satellite info data */
class GPS_Sat_Info
{
public:
	struct satellite_info_s 	_data;
};


class GPS {

public:

	GPS(char* uart_name);

	void main_loop();

private:

	SerialPort*	_serial;	///< instance of the serial port
	std::ofstream _logfile;	///< log file for saving all the raw binary data

	bool	_healthy;		///< flag to signal if the GPS is ok
	unsigned _baudrate;

	gps_driver_mode_t		_mode;			///< current mode
	GPSHelper::Interface  	_interface;   	///< interface
	GPSHelper*				_helper;		///< instance of GPS parser

	struct vehicle_gps_position_s _report_gps_pos;
	GPS_Sat_Info				*_sat_info;				///< instance of GPS sat info data object
	struct satellite_info_s		*_p_report_sat_info;	///< pointer to uORB topic for satellite info


	void publish();
	void publishSatelliteInfo();

	int readFromSerial(uint8_t *buf, size_t buf_length, int timeout);
	int writeToSerial(void *buf, size_t buf_length);

	/**
	 * callback from the driver for the platform specific stuff
	 * this is how we handle reading/writing to the GPS device
	 *
	 * see the GPSDriver Helper for details.
	 */
	static int callback(GPSCallbackType type, void *data1, int data2, void *user);


};

GPS::GPS(char* uart_name) :
_logfile("log.dat", std::ofstream::binary),
_healthy(false),
_mode(GPS_DRIVER_MODE_UBX),
_interface(GPSHelper::Interface::UART),
_helper(nullptr),
_report_gps_pos{},
_sat_info(nullptr),
_p_report_sat_info(nullptr)
{
	// create the serial device
	_serial = new SerialPort(true, uart_name);

	if (_logfile) {
		cout << "we have a log file!" << endl;
		//_logfile << "test\n";
	} else {
		cout << "no logfile!" << endl;
	}

	/* create satellite info data object if requested */
	_sat_info = new GPS_Sat_Info();
	_p_report_sat_info = &_sat_info->_data;
	memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
}

int GPS::readFromSerial(uint8_t *buf, size_t buf_length, int timeout) {

	fd_set set;
	struct timeval to_struct;
	int rv;

	FD_ZERO(&set); /* clear the set */
	FD_SET(_serial->fd, &set); /* add our file descriptor to the set */

	to_struct.tv_sec = 0;
	to_struct.tv_usec = timeout*1000;

	rv = select(_serial->fd + 1, &set, NULL, NULL, &to_struct);
	if(rv == -1) {
		// error occured on selecting
		//cout << "select error" << endl;
		return -1;
	} else if(rv == 0) {
		// timeout occured
		//cout << "select timeout" << endl;
		return 0;
	} else {
		int res = read(_serial->fd, buf, buf_length); /* there was data to read */
		if (res > 0) {
			_logfile.write(reinterpret_cast<char*>(buf), res);
		}
		return res;
	}
}

int GPS::writeToSerial(void *data1, size_t buf_length) {
	// write the info out to the ublox receiver
	int len = write(_serial->fd, data1, buf_length);
	//cout << "wrote " << len << endl;

	// write the same data to a log file
	//_cfgfile.write((char *)data1, buf_length);

	// return the number of bytes written
	return len;
}

int GPS::callback(GPSCallbackType type, void *data1, int data2, void *user) {
	GPS *gps = (GPS *)user;

	//cout << "callback type: " << (int) type << endl;

	int num_read = 0;
	switch (type) {
	case GPSCallbackType::readDeviceData:
		// TODO: decide how to use the timeout
		// int timeout = *((int *)data1)
		//cout << "reading from device (" << data2 << ")" << endl;
		usleep(10000);

		//num_read = read(gps->_serial->fd, (uint8_t *)data1, data2);
		num_read = gps->readFromSerial((uint8_t *)data1, data2, *((int *)data1));
		//cout << "read " << num_read << " bytes" << endl;

		return num_read;

	case GPSCallbackType::writeDeviceData:
		cout << "writing to device" << endl;

		//return write(gps->_serial->fd, data1, (size_t)data2);
		return gps->writeToSerial(data1, (size_t)data2);

	case GPSCallbackType::setBaudrate:
		cout << "setting baudrate to " << data2 << endl;
		if (gps->_serial->set_baudrate(data2)) {
			return 0;
		} else {
			return -1;
		}

	case GPSCallbackType::gotRTCMMessage:
		/* not used */
		break;

	case GPSCallbackType::surveyInStatus:
		/* not used */
		break;

	case GPSCallbackType::setClock:
		/* not used for now */
		break;
	}

	return 0;
}

void GPS::publish() {
	// TODO: make output to terminal a setting
	/*
	cout << "position: " << endl;
	cout << "\ttime:\t" << _report_gps_pos.time_utc_usec << endl;
	cout << "\tfix type:\t" << (int)_report_gps_pos.fix_type << endl;
	cout << "\tlat:\t" << _report_gps_pos.lat << endl;
	cout << "\tlon:\t" << _report_gps_pos.lon << endl;
	cout << "\talt:\t" << _report_gps_pos.alt << endl;
	cout << "\talt ellipsoid:\t" << _report_gps_pos.alt_ellipsoid << endl;
	*/
}

void GPS::publishSatelliteInfo() {
	// TODO: make output to terminal a setting
	// TODO: simply print out the sat info
	/*
	cout << "sat message received" << endl;
	*/
}

void GPS::main_loop() {

	// TODO: figure out what these do
	//uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (true) {

		// some management
		if (_helper != nullptr) {
			delete (_helper);
			_helper = nullptr;
		}

		switch (_mode) {
		case GPS_DRIVER_MODE_NONE:
			_mode = GPS_DRIVER_MODE_UBX;

		//no break
		case GPS_DRIVER_MODE_UBX:
			_helper = new GPSDriverUBX(_interface, &GPS::callback, this, &_report_gps_pos, _p_report_sat_info);
			break;

		case GPS_DRIVER_MODE_MTK:
			cout << "MTK not supported!" << endl;
			//_helper = new GPSDriverMTK(&GPS::callback, this, &_report_gps_pos);
			break;

		case GPS_DRIVER_MODE_ASHTECH:
			cout << "ASHTECH not supported!" << endl;
			//_helper = new GPSDriverAshtech(&GPS::callback, this, &_report_gps_pos, _p_report_sat_info);
			break;

		default:
			break;
		}


		/* the Ashtech driver lies about successful configuration and the
		 * MTK driver is not well tested, so we really only trust the UBX
		 * driver for an advance publication
		 */
		if (_helper && _helper->configure(_baudrate, GPSHelper::OutputMode::GPS) == 0) {

			/* reset report */
			memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));

			if (_mode == GPS_DRIVER_MODE_UBX) {
				/* Publish initial report that we have access to a GPS,
				 * but set all critical state fields to indicate we have
				 * no valid position lock
				 */

				/* reset the timestamp for data, because we have no data yet */
				_report_gps_pos.timestamp = 0;
				_report_gps_pos.timestamp_time_relative = 0;

				/* set a massive variance */
				_report_gps_pos.eph = 10000.0f;
				_report_gps_pos.epv = 10000.0f;
				_report_gps_pos.fix_type = 0;

				publish();

				/* GPS is obviously detected successfully, reset statistics */
				_helper->resetUpdateRates();
			}

			int helper_ret;

			while ((helper_ret = _helper->receive(TIMEOUT_5HZ)) > 0) {

				if (helper_ret & 1) {
					publish();

					last_rate_count++;
				}

				if (_p_report_sat_info && (helper_ret & 2)) {
					publishSatelliteInfo();
				}

				/* measure update rate every 5 seconds */
				// TODO: need to handle the time stuff here...
				/*
				if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
					float dt = (float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f;
					_rate = last_rate_count / dt;
					_rate_rtcm_injection = _last_rate_rtcm_injection_count / dt;
					last_rate_measurement = hrt_absolute_time();
					last_rate_count = 0;
					_last_rate_rtcm_injection_count = 0;
					_helper->storeUpdateRates();
					_helper->resetUpdateRates();
				}
				*/

				if (!_healthy) {
					// Helpful for debugging, but too verbose for normal ops
					const char *mode_str = "unknown";

					switch (_mode) {
					case GPS_DRIVER_MODE_UBX:
						mode_str = "UBX";
						break;

					case GPS_DRIVER_MODE_MTK:
						mode_str = "MTK";
						break;

					case GPS_DRIVER_MODE_ASHTECH:
						mode_str = "ASHTECH";
						break;

					default:
						break;
					}

					printf("module found: %s", mode_str);
					_healthy = true;
				}
			}

			if (_healthy) {
				printf("GPS module lost");
				_healthy = false;
				//_rate = 0.0f;
				//_rate_rtcm_injection = 0.0f;
			}
		}
	}
}






// the main script
int main(int argc, char **argv) {

	cout << "starting...\n";

	
	char* uart_name = (char*) "";
	int baudrate = 9600;

	// retrieve the user input (serial port)
	for (int i = 1; i < argc; i++) {

		/* UART device ID */
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];
			} else {
				cout << "no device passed\n";
				return -1;
			}
		}

		/* UART device ID */
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baudrate") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);
			}
		}
	}

	// connect to the serial port
	
	bool verbose = true;
	
	// create the GPS device
	GPS* gps = new GPS(uart_name);


	// now run the main task
	gps->main_loop();




}