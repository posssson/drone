#ifndef DEF_SONAR
#define DEF_SONAR
#include <time.h>
#include <iostream>
#include <wiringPi.h>
#include <unistd.h>

class Sonar
{
  public:
    Sonar();
    void init(int trigger, int echo);
    double distance(float timeout);

  private:
    int trigger;
    int echo;
    double distanceMeters;
    float travelTimeSec;
    struct timespec now;
    struct timespec time;
    struct timespec startTimeSec;
    struct timespec endTimeSec;
};

#endif
