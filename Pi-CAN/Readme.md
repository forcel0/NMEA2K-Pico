# NMEA2000_PICO
This library provides Raspberry Pi Pico boards w/ an Adafruit PiCow CAN hat driver for the NMEA2000 library
This library is in eearly stage developement. End goal is to support other CAN hats for the Pi Pico

See https://github.com/ttlappalainen/NMEA2000.

## Usage


    #include "NMEA2000.h"
    #include "N2kMessages.h"
    #include "NMEA2000_PICO.h"

    tNMEA2000_pico NMEA2000;

    void setup() {
      NMEA2000.Open();
    }

    void loop() {
	  NMEA2000.ParseMessages();
    }

See the [NMEA2000/Examples](https://github.com/ttlappalainen/NMEA2000/tree/master/Examples) for more examples. They are all compatible with this library.
See also See https://github.com/forcel0/NMEA2K-Pico for a project using NMEA2000 library and NMEA2000_PICO

## Changes
    05.11.2025
	
	- Initial commit

## License

    The MIT License

    Copyright (c) 2020-2023 Timo Lappalainen

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
