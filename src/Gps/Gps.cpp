
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include "std_msgs/String.h"
//#include "serial.h"
#include <wiringSerial.h>
using namespace std;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int messagerecu =0;
int fd,i ;
int nbr;
int length = 0;
string dollar_test = "$";
string gps_message;
string char_gps;

//SimpleSerial * serial;
// %Tag(CALLBACK)%
void gps();
void gps()
{



	i=0;
	while (1) // Tant que la valeur est pas $
	{
		char_gps = (char)serialGetchar(fd);
		if (char_gps.compare(dollar_test)==0)
		{
			char_gps.clear();
			break;
		}
		gps_message += (char_gps);
		char_gps.clear();
	}

	fflush (stdout) ;

	std::string delimiter = ",";

	// On parse les résultats
	size_t pos = 0;
	std::string token;
	while ((pos = gps_message.find(delimiter)) != std::string::npos) {
	    token = gps_message.substr(0, pos);
	    //std::cout << token << std::endl;
	    gps_message.erase(0, pos + delimiter.length());
	}


	//cout<<gps_message<<endl;
	gps_message.clear();

}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{

	ros::init(argc, argv, "gps");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("gps", 1);

	ROS_INFO("Init GPS ok");
	//serial = new SimpleSerial("/dev/ttyAMA0", 9600, 100);
	fd = serialOpen("/dev/ttyAMA0", 9600);
	while (ros::ok())
	{

		sleep(0.2);
		gps();
		//ROS_INFO("fin gps");
		ros::spinOnce();


	}



	return 0;
}
// %EndTag(FULLTEXT)%

