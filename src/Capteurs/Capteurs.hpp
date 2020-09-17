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
//#include "MPU9250pio.h"
#include "../Common/LowPassFilter2p.hpp"
#include "../Common/Biquad.h"
#include <wiringPi.h>
#include "MadgwickAHRS.hpp"
#include <errno.h>
#include <thread>
#include <chrono>
#include "../Common/Realtime.h"


#define beta 0.6f
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

drone::Capteurs_msg msg_attitude;
ros::Publisher _pub_msg_attitude;

//MPU9250 mpu(0x68);
MPU9250 mpu(0 , 15000000);
//MPU9250pio mpu(0 , 20000000);



//MPU9250 mpu(0 , 1000000);
//MPU9250 mpu(0 , 100000);
float ax, ay, az;
float Last_ax = 0, Last_ay = 0, Last_az = -9.1;
float gx, gy, gz;
float Last_gx = 0, Last_gy = 0, Last_gz = 0;
float mx, my, mz;
float Last_mx, Last_my, Last_mz;
float alpha = 0.02;
float alpha_Mag = 0.5;
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
temps_attente_nanosleep,ancien_temps_test,temps_actuel_test;
float temps_recup_altitude = 0,temps_publication_message = 0,temps_test = 0;
float frequence = 10;
int frequenceSet = 1000;
int correctFrequency;
float time_loop = 1/(float)frequenceSet;
float sec_to_nano = 1000000000;
int fd;
int it = 0;
bool print = true;
float rad2deg = 57.295779; 
float deg2rad = 0.017453;
float magx,magy,yaw;

int compteur1 = 0;
int compteur2 = 0;
float offsetGx = -0.3205;
float offsetGy = -1.632;
float offsetGz = 0.5407;
float offsetAx = -0.1334;
float offsetAy = 0.1;
float offsetAz = 0.494;
float offsetMx = -18.0;
float offsetMy = -11.0;
float offsetMz = -1.0;

void callback1(const ros::TimerEvent&);
void setup();
void calibrate_value(int i);
void ActualisationAccel();
void ActualisationCapteurs();
void ActualisationMagn();

LowPassFilter2p lowFilterGx(frequenceSet, 80);
LowPassFilter2p lowFilterGy(frequenceSet, 80);
LowPassFilter2p lowFilterGz(frequenceSet, 80);
LowPassFilter2p lowFilterGx2(frequenceSet, 80);
LowPassFilter2p lowFilterGy2(frequenceSet, 80);
LowPassFilter2p lowFilterGz2(frequenceSet, 80);

LowPassFilter2p lowFilterAx(frequenceSet, 80);
LowPassFilter2p lowFilterAy(frequenceSet, 80);
LowPassFilter2p lowFilterAz(frequenceSet, 80);
LowPassFilter2p lowFilterAx2(frequenceSet, 80);
LowPassFilter2p lowFilterAy2(frequenceSet, 80);
LowPassFilter2p lowFilterAz2(frequenceSet, 80);

Biquad BiquadGx;
Biquad BiquadGy;
Biquad BiquadGz;

Biquad BiquadAx;
Biquad BiquadAy;
Biquad BiquadAz;