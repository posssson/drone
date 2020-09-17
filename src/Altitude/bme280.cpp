/***************************************************************************
Modified BSD License
====================
Copyright © 2016, Andrei Vainik
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the organization nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
This piece of code was combined from several sources
https://github.com/adafruit/Adafruit_BME280_Library
https://cdn-shop.adafruit.com/datasheets/BST-BME280_DS001-10.pdf
https://projects.drogon.net/raspberry-pi/wiringpi/i2c-library/
Compensation functions and altitude function originally from:
https://github.com/adafruit/Adafruit_BME280_Library/blob/master/Adafruit_BME280.cpp
***************************************************************************
This is a library for the BME280 humidity, temperature & pressure sensor
Designed specifically to work with the Adafruit BME280 Breakout
----> http://www.adafruit.com/products/2650
These sensors use I2C or SPI to communicate, 2 or 4 pins are required
to interface.
Adafruit invests time and resources providing this open source code,
please support Adafruit andopen-source hardware by purchasing products
from Adafruit!
Written by Limor Fried & Kevin Townsend for Adafruit Industries.
BSD license, all text above must be included in any redistribution
***************************************************************************
****************************************************************************/


#include "bme280.hpp"


bme280::bme280(int i2c_address){
	I2CADDR = i2c_address; // I2C address
}

int bme280::begin(){
	if((device = open(BUS, O_RDWR)) < 0)
	{
		printf("Failed to open the i2c bus. \n");
		return(-1);
	}
	// get I2C device
	ioctl(device, I2C_SLAVE, I2CADDR);

	// read 24 bytes of calibration data from address(0x88)
	reg[0] = 0x88;
	write(device, reg, 1);
	if(read(device, data, 24) != 24)
	{
		printf("Unable to read data from i2c bus\n");
		return(-2);
	}

	// temp coefficents
	T[0] = data[1] * 256 + data[0];
	T[1] = data[3] * 256 + data[2];
	if(T[1] > 32767) { T[1] -= 65536; }
	T[2] = data[5] * 256 + data[4];
	if(T[2] > 32767) { T[2] -= 65536; }

	// pressure coefficents
	P[0] = data[7] * 256 + data[6];
	for (i = 0; i < 8; i++)
	{
		P[i+1] = data[2*i+9]*256 + data[2*i+8];
		if(P[i+1] > 32767) { P[i+1] -= 65536; }
	}

	// Select control measurement register(0xF4)
	// normal mode, temp and pressure oversampling rate = 1(0x27)
	config[0] = 0xF4;
	config[1] = 0xB7;
	write(device, config, 2);

	// select config register(0xF5)
	// stand_by time = 0.5 ms(0x0E)
	config[0] = 0xF5;
	config[1] = 0x00;
	write(device, config, 2);
	return 1;
	
}


void bme280::getRawData( bme280_raw_data *raw) {
	// Read 6 bytes of data from register(0xF7)
	reg[0] = 0xF7;
	write(device, reg, 1);
	if(read(device, data, 6) != 6)
	{
		printf("Unable to read data from i2c bus\n");
		exit(1);
	}

	// Convert pressure and temperature data to 19-bits
	long adc_p = (((long)data[0] << 12) + ((long)data[1] << 4) + (long)(data[2] >> 4));
	long adc_t = (((long)data[3] << 12) + ((long)data[4] << 4) + (long)(data[5] >> 4));

	// temperature offset calculations
	temp1 = (((double)adc_t)/16384.0 - ((double)T[0])/1024.0)*((double)T[1]);
	temp3 = ((double)adc_t)/131072.0 - ((double)T[0])/8192.0;
	temp2 = temp3*temp3*((double)T[2]);
	temperature = (temp1 + temp2)/5120.0;

	// pressure offset calculations
	press1 = ((temp1 + temp2)/2.0) - 64000.0;
	press2 = press1*press1*((double)P[5])/32768.0;
	press2 = press2 + press1*((double)P[4])*2.0;
	press2 = (press2/4.0) + (((double)P[3])*65536.0);
	press1 = (((double) P[2])*press1*press1/524288.0 + ((double) P[1])*press1)/524288.0;
	press1 = (1.0 + press1/32768.0)*((double)P[0]);
	press3 = 1048576.0 - (double)adc_p;
	if (press1 != 0.0)                                 // avoid error: division by 0
	{
		press3 = (press3 - press2/4096.0)*6250.0/press1;
		press1 = ((double) P[8])*press3*press3/2147483648.0;
		press2 = press3 * ((double) P[7])/32768.0;
		pressure = (press3 + (press1 + press2 + ((double)P[6]))/16.0)/100;
	}
	else
	{ pressure = 0.0; }
	raw->temperature=temperature;
	raw->pressure=pressure;
}

float bme280::getAltitude(float pressure) {
	// Equation taken from BMP180 datasheet (page 16):
	//  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

	// Note that using the equation from wikipedia can give bad results
	// at high altitude.  See this thread for more information:
	//  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

	return 44330.0 * (1.0 - pow(pressure / MEAN_SEA_LEVEL_PRESSURE, 0.190294957));
}
