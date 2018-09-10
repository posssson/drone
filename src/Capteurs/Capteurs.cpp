#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone/Capteurs_msg.h"
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <fstream>
using namespace std;
#include "bme280.c"
#include "I2Cdev.h"
#include "MPU9250.h"
#include <wiringPi.h>
#include "../Common/LowPassFilter2p.hpp"
#include "../Common/Biquad.h"
#include "MadgwickAHRS.c"

#include <errno.h>
#include <wiringPiI2C.h>



// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

MPU9250 mpu(0x68);
math::LowPassFilter2p LPFAcc_x(1000, 50);
math::LowPassFilter2p LPFAcc_y(1000, 50);
math::LowPassFilter2p LPFAcc_z(1000, 50);
math::LowPassFilter2p LPFGyro_x(1000, 50);
math::LowPassFilter2p LPFGyro_y(1000, 50);
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
int i = 1;
float gyroScale = 131;
float arx, ary, arz, grx, gry, grz, gsx, gsy, gsz;
float timePrev = 0, timet = 0, timeStep = 0;
float ypr[3];
struct timeval tv;
float anglex = 0, angley = 0, anglez = 0;
float angle_x = 0, angle_y = 0, angle_z = 0;
float temps_proc = 0;
float anglex_prec = 0, angley_prec = 0, anglez_prec = 0;
float vitx = 0, vity = 0;
float angle_accel_x = 0, angle_accel_y = 0;
struct timespec time_actuel, ancien_temps, temps_attente,
		temps_attente_nanosleep;
float R12=0;
float R22 = 0;
float R23=0;
float R31=0;
float R32 =0;
float R33 = 0;
float frequence =0;
Biquad *lpFilterX = new Biquad(); // create a Biquad, lpFilter;
Biquad *lpFilterY = new Biquad(); // create a Biquad, lpFilter;
Biquad *lpFilterZ = new Biquad(); // create a Biquad, lpFilter;

int32_t t_fine;
float t ; // C
float p; // hPa
float h;       // %
float a;                         // meters


int fd;
int it = 0;
bme280_raw_data raw;
bme280_calib_data cal;


void setup() {
	temps_attente.tv_sec = 0;
	temps_attente.tv_nsec = 10;

	lpFilterX->setBiquad(bq_type_lowpass, 0.00001, 0.707, 0);
	lpFilterY->setBiquad(bq_type_lowpass, 0.00001, 0.707, 0);
	lpFilterZ->setBiquad(bq_type_lowpass, 0.00001, 0.707, 0);

// initialize device
	printf("Initializing I2C devices...\n");
//mpu.initialize();

// verify connection
	printf("Testing device connections...\n");
	int reset = 0;
	mpu.begin(ACCEL_RANGE_8G, GYRO_RANGE_1000DPS);

//Altitude
	  fd = wiringPiI2CSetup(BME280_ADDRESS);
	  if(fd < 0) {
	    printf("Device not found");
	  }


	  readCalibrationData(fd, &cal);

	  wiringPiI2CWriteReg8(fd, 0xf2, 0x01);   // humidity oversampling x 1
	  wiringPiI2CWriteReg8(fd, 0xf4, 0x25);   // pressure and temperature oversampling x 1, mode normal


}

static void printData() {

	// print the data
	/*  printf("ax  =%6.6f\n", ax);
	 printf("ay  =%6.6f\n", ay);
	 printf("az  =%6.6f\n", az);

	 printf("gx   =%6.6f\n", gx);
	 printf("gy   =%6.6f\n", gy);
	 printf("gz   =%6.6f\n", gz);

	 printf("mx  =%6.6f\n", mx);
	 printf("my  =%6.6f\n", my);
	 printf("mz  =%6.6f\n", mz);*/
	//printf("anglez  =%6.6f\n", anglez);
	printf("temps_proc  =%f\n", 1/temps_proc);
	printf("anglex   =%6.6f    =%6.6f\n", anglex,angle_x);
	printf("angley   =%6.6f    =%6.6f\n", angley,angle_y);
	printf("anglez   =%6.6f    =%6.6f\n", anglez,angle_z);


	  //readCalibrationData(fd, &cal);
	  //wiringPiI2CWriteReg8(fd, 0xf2, 0x01);   // humidity oversampling x 1
	   wiringPiI2CWriteReg8(fd, 0xf4, 0x25);   // pressure and temperature oversampling x 1, mode normal
	   getRawData(fd, &raw);
	   t_fine = getTemperatureCalibration(&cal, raw.temperature);
	   //t = compensateTemperature(t_fine); // C
	   p = compensatePressure(raw.pressure, &cal, t_fine) / 100; // hPa
	   //h = compensateHumidity(raw.humidity, &cal, t_fine);       // %
	   a = getAltitude(p);                         // meters

	  printf("{\"sensor\":\"bme280\", \"humidity\":%.2f, \"pressure\":%.2f,"
	    " \"temperature\":%.2f, \"altitude\":%.2f}\n",
	    h, p, t, a);
}
void loop() {

	clock_gettime(CLOCK_REALTIME, &time_actuel);
	temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec)
			+ (time_actuel.tv_nsec - ancien_temps.tv_nsec) / 1000000000.0;

	/*while (temps_proc < 0.001)
	 {
	 clock_gettime( CLOCK_REALTIME, &time_actuel);
	 temps_proc =(time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;
	 nanosleep(&temps_attente,&temps_attente_nanosleep);
	 printf("temps_proc        %f\n", temps_proc);
	 }*/
	ancien_temps = time_actuel;

	mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

// calibration
// TODO mettre dans .h
	gx -= 1.80;
	gy += 1.55;
	gz -= 0.78;
	ax = ax * 0.997505 - 0.73333;
	ay = ay * 0.998909 - 1.07523;
	az = az * 0.986718 - 0.897756;

	/////////////////////
	frequence = 1/temps_proc;

	MadgwickAHRSupdateIMU(gx, gy,gz,ax, ay, az,frequence);
 /////////////////////////////

// Application filtre
	ax = LPFAcc_x.apply(ax);
	ay = LPFAcc_y.apply(ay);
	az = LPFAcc_z.apply(az);
	gx = LPFGyro_x.apply(gx);
	gy = LPFGyro_y.apply(gy);

// Calcul angle euler avec filtre compensatoire (fonctionne)
	angle_accel_x = (atan((float) -ax / sqrt((float) az * (float) az))) * 180.0
			/ M_PI;
	angle_accel_y = (atan((float) ay / sqrt((float) az * (float) az))) * 180.0
			/ M_PI;

	anglex = 0.98 * (anglex - (float) (gy) * temps_proc) + 0.02 * angle_accel_x;
	angley = 0.98 * (angley - (float) (gx) * temps_proc) + 0.02 * angle_accel_y;


	///////////////////////////
// Calcul du lacet avec le magnÃ©tometre
	anglez = 0;

	angle_y =- atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)* 57.29578f-180;
	angle_x = asin(-2.0f * (q1*q3 - q0*q2))* 57.29578f;
	angle_z = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)* 57.29578f;


	if (temps_proc != 0) {
		vitx = (anglex - anglex_prec) / temps_proc;
		vity = (angley - angley_prec) / temps_proc;
	}

	anglex_prec = anglex;
	angley_prec = angley;

	if (abs(anglex) > 380 || abs(angley) > 380) {
		anglex = 0;
		angley = 0;
	}
}

void recuperation_initialisation(const std_msgs::String::ConstPtr& msg,
		int *recu_init) // vÃ©rification initialisation
		{
	*recu_init = 1;
	ROS_INFO("Init recu");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "capteur");
	ros::NodeHandle n;

	setup();

	int recu_init = 0;

	ofstream fichier("/home/pi/drone_ws/src/drone/src/donnees_accelero.txt",
			ios::out | ios::trunc);

	boost::function<void(const std_msgs::String::ConstPtr& msg)> temp =
			boost::bind(recuperation_initialisation, _1, &recu_init);
	ros::Subscriber sub = n.subscribe("initialisation", 1, temp);
	drone::Capteurs_msg msg_attitude;
	ROS_INFO("Capteurs");

	ros::Publisher _pub_msg_attitude = n.advertise < drone::Capteurs_msg
			> ("capteurs", 1);

	while (ros::ok()) {
		loop();
		if (recu_init == 1) {
			fichier << " " << ax << " " << ay << " " << az << " " << gy << " "
					<< gx << " " << temps_proc << endl;

//MDP
			msg_attitude.x = anglex; // tangage
			msg_attitude.y = angley; // roulis
			msg_attitude.z = anglez; // lacet calculer avec le magnÃ©tometre
			msg_attitude.vx = -gy;
			msg_attitude.vy = -gx;
			printData();
			_pub_msg_attitude.publish(msg_attitude);
		}
		ros::spinOnce();
	}

// %EndTag(SPIN)%

	return 0;
}
// %EndTag(FULLTEXT)%
