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
#include "MadgwickAHRS.hpp"
#include <errno.h>


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

MPU9250 mpu(0x68);
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

float roll,pitch ,heading;
Madgwick filter,filter_magneto;
int i = 1;
float timePrev = 0, timet = 0, timeStep = 0;
struct timeval tv;
float anglex = 0, angley = 0, anglez = 0;
float angle_x = 0, angle_y = 0, angle_z = 0;
float temps_proc = 0;
float anglex_prec = 0, angley_prec = 0, anglez_prec = 0;
float vitx = 0, vity = 0;
struct timespec time_actuel, ancien_temps, temps_attente,
		temps_attente_nanosleep;
float temps_recup_altitude = 0,temps_publication_message=0;
float frequence = 10;
float sec_to_nano = 1000000000;
int fd;
int it = 0;
bool print =true;
float betaRollPitch= 0.2, betaYaw = 1.5;
void setup();
void calibrate_value();


