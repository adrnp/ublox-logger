# Ublox Logger #

This currently contains a fairly simple script to log the raw ublox binary data from a ublox receiver to a file.  Outputs the raw data to a file called `log.ubx`.  This can then be imported into u-center or used in any way that you may be using a `.ubx` file from u-center.

**NOTE: this has only been tested with a ublox NEO-M8T chip.  This also configures the receiver to receive Galileo signals in addition to GPS and GLONASS.**

## Building Code ##

The cmake build system is used for building the code, so make sure that you have `cmake` installed.  Once installed, building the code can be done in the following steps:

 1. make a build file directory (if not already made): `mkdir build`

 2. enter the build directory and run cmake: `cd build && cmake ..`

 3. run make from the build directory: `make`

There should now be an executable in the `build/` directory called `ublox_logger`.

## Running Code ##

To run the code, call the executable that was created and provide the following two inputs: the port and the baud rate.

For example, to log the data from a ublox receiver connected on `/dev/ttyUSB0` at a baud rate of `115200`, you would run the following:

```sh
$ ./ublox_logger -d /dev/ttyUSB0 -b 115200
```