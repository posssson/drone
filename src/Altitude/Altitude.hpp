#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone/Altitude_msg.h"
#include "drone/Gaz_msg.h"
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
using namespace std;
#include "bme280.hpp"
#include "I2Cdev.h"
#include <wiringPi.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>
#include <wiringPi.h>
#include "libSonar.h"

#define PIN_ECHO 26 //25
#define PIN_PULSE 21//29

bme280 bme(0x76);
Sonar sonar;
bme280_raw_data value;
void recuperation_gaz(const drone::Gaz_msg::ConstPtr& _msg, float *_gaz);
int dig_T1,dig_T2,dig_T3,dig_P1,dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9;
float gaz = 0;
float distance_ultrason;
struct timespec start_echo, stop_echo, diff_echo;
double echo_time;
int i = 1;
float timePrev = 0, timet = 0, timeStep = 0;
struct timeval tv;
float temps_proc = 0;
struct timespec time_actuel, ancien_temps, temps_attente,
temps_attente_nanosleep;
float temps_recup_altitude = 0;
float frequence = 0;
float altitude_baro = 0;
float correction_altitude = 0;
float sec_to_nano = 1000000000;
int32_t t_fine;
bool print = true;
float t; // C
float h;  // %
int file;
double pressure,p;
char reg[1];
char config[2];
char data[24];