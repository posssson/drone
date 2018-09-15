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

#define PIN_ECHO 16
#define PIN_PULSE 20
