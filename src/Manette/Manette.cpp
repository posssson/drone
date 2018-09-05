#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <wiringPi.h>
#include <pigpio.h>
#include <pthread.h>
#include <time.h>
#include <softPwm.h>
#include "drone/Consigne_msg.h"
using namespace std;
// %EndTag(CALLBACK)%
// PIN MAP pinmappingsrpi
void aFunction(int gpio, int level, uint32_t tick);




int main(int argc, char **argv)
{
  ros::init(argc, argv, "Manette");
  ros::NodeHandle n;
  ros::Publisher _pub_msg_consigne = n.advertise<drone::Consigne_msg>("manette", 1);
    if (gpioInitialise() < 0)
    {
        printf ("BUG\n") ;
    }
    else
    {
        printf ("OK\n") ;
    }
	sleep(10);
    // call aFunction whenever GPIO 4 changes state
    gpioSetAlertFunc(13, gaz);
	gpioSetAlertFunc(23, tangage);
	gpioSetAlertFunc(24, lacet);
	gpioSetAlertFunc(25, roulis);

	drone::Consigne_msg msg;
	while (1)
	{
		usleep(10);
		msg.gaz = temps_haut_gaz;
		msg.tangage = temps_haut_tangage;
		msg.roulis = temps_haut_roulis;
		msg.lacet = temps_haut_lacet;
		_pub_msg_consigne.publish(msg);
	}
     printf ("fin\n") ;
         ros::spin();

// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

