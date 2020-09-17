#include <iostream>
#include <wiringPi.h>
#include "libSonar.h"
#include <stdio.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

using namespace std;
#define PIN_ECHO 26
#define PIN_PULSE 16
int trigger = 16;
int echo = 26;

int main()
{
    if (wiringPiSetupGpio() == -1)
        return -1;

    Sonar sonar;
    sonar.init(trigger, echo);

    while(1){
        cout << "Distance is " << sonar.distance(0.03) << " cm." << endl;
    }
}