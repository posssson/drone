
#include "libSonar.h"

Sonar::Sonar(){}

void Sonar::init(int trigger, int echo)
{
    wiringPiSetupGpio() ;
    this->trigger=trigger;
    this->echo=echo;
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trigger, LOW);
    delay(500);
}

double Sonar::distance(float timeout)
{
    usleep(10);
    digitalWrite(trigger, HIGH);
    usleep(10000);
    digitalWrite(trigger, LOW);
    clock_gettime(CLOCK_REALTIME, &now);
    clock_gettime(CLOCK_REALTIME, &time);
    while (digitalRead(echo) == LOW && ((time.tv_sec - now.tv_sec) + (time.tv_nsec - now.tv_nsec)/1000000000.0) <timeout)
    {
        clock_gettime(CLOCK_REALTIME, &time);
    }
    clock_gettime(CLOCK_REALTIME, &time);
    if (((time.tv_sec - now.tv_sec) + (time.tv_nsec - now.tv_nsec)/1000000000.0) > timeout)
    {
        return -1;
    }
    clock_gettime(CLOCK_REALTIME, &startTimeSec);
    while ( digitalRead(echo) == HIGH );
    clock_gettime(CLOCK_REALTIME, &endTimeSec);


    if(((endTimeSec.tv_sec - startTimeSec.tv_sec) + (endTimeSec.tv_nsec - startTimeSec.tv_nsec)/1000000000.0) < timeout)
    {
        travelTimeSec = (endTimeSec.tv_sec - startTimeSec.tv_sec) + (endTimeSec.tv_nsec - startTimeSec.tv_nsec)/1000000000.0;
        distanceMeters = (travelTimeSec*340.29)/2;
    }
    else
    {
        distanceMeters=-1; 
    }
    return distanceMeters;
}

