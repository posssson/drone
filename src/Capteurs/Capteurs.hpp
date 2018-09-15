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
#include "I2Cdev.h"
#include "MPU9250.h"
#include <wiringPi.h>
#include "../Common/LowPassFilter2p.hpp"
#include "../Common/Biquad.h"
#include "MadgwickAHRS.c"
#include <errno.h>


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
float temps_recup_altitude = 0;
float frequence = 0;
Biquad *lpFilterX = new Biquad(); // create a Biquad, lpFilter;
Biquad *lpFilterY = new Biquad(); // create a Biquad, lpFilter;
Biquad *lpFilterZ = new Biquad(); // create a Biquad, lpFilter;

float sec_to_nano = 1000000000;
int fd;
int it = 0;

void setup();
