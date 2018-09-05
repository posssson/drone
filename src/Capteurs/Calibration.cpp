#include <stdio.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"

#include <wiringPi.h>
#include "MPU9250.h"
#include <iostream>
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
using namespace std;
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 mpu(0x68);

float ax, ay, az,mx,my,mz;
float gx , gy, gz;


void setup() {

	// initialize device
	printf("Initializing I2C devices...\n");
	//mpu.initialize();
	
	// verify connection
	printf("Testing device connections...\n");
	//printf("%d   \n",mpu.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS));// ? "MPU9250 connection successful\n" : "MPU9250 connection failed\n");
while((mpu.begin(ACCEL_RANGE_8G,GYRO_RANGE_1000DPS))!= 0)
{
	// reset the MPU9250
	printf("reset the MPU9250...\n");

    // wait for oscillators to stabilize
    delay(100);
}
}

void loop() {
	// read raw accel/gyro measurements from device
	mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz,&mx, &my, &mz);
	/*	std::cout << "mx = " << gx << std::endl;
		std::cout << "my = " << gy << std::endl;
		std::cout << "mz = " << gz << std::endl;*/
	// these methods (and a few others) are also available
	//accelgyro.getAcceleration(&ax, &ay, &az);
	//accelgyro.getRotation(&gx, &gy, &gz);

	// display accel/gyro x/y/z values
	//printf("a/g: %6hd %6hd %6hd   %6hd %6hd %6hd\n", ax, ay, az, gx, gy, gz);
}

int main()
{
	setup();

	std::cout << "On ne bouge pas!" << std::endl;
	float somme_x_mpu = 0, somme_y_mpu = 0, somme_z_mpu = 0;
	float somme_mx_mpu = 0, somme_my_mpu = 0, somme_mz_mpu = 0;
	float moyenne_x_mpu, moyenne_y_mpu, moyenne_z_mpu;
	float moyenne_mx_mpu, moyenne_my_mpu, moyenne_mz_mpu;
	int compte;
	int count = 0;
	for (compte = 1; compte < 5000; compte++)
	{
		usleep(10);
		loop();
		/*std::cout << "gx = " << gx << std::endl;
		std::cout << "gy = " << gy << std::endl;
		std::cout << "gz = " << gz << std::endl;*/
	//	gx += 0;
	//	gy += 0;
	//	gz += 0;
		somme_x_mpu += gx;
		somme_y_mpu += gy;
		somme_z_mpu += gz;
		somme_mx_mpu += mx;
		somme_my_mpu += my;
		somme_mz_mpu += mz;
		count++;
	}
	moyenne_x_mpu = somme_x_mpu / (float)count;
	moyenne_y_mpu = somme_y_mpu / (float)count;
	moyenne_z_mpu = somme_z_mpu / (float)count;
	moyenne_mx_mpu = somme_mx_mpu / (float)count;
	moyenne_my_mpu = somme_my_mpu / (float)count;
	moyenne_mz_mpu = somme_mz_mpu / (float)count;
	std::cout << "gyrox_offset_MPU = " << moyenne_x_mpu << std::endl;
	std::cout << "gyroy_offset_MPU = " << moyenne_y_mpu << std::endl;
	std::cout << "gyroz_offset_MPU = " << moyenne_z_mpu << std::endl;
	std::cout << "mag_offset_MPU = " << moyenne_mx_mpu << std::endl;
	std::cout << "mag_offset_MPU = " << moyenne_my_mpu << std::endl;
	std::cout << "mag_offset_MPU = " << moyenne_mz_mpu << std::endl;
}
