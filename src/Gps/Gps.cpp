
// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include "std_msgs/String.h"
#include "serial.h"
using namespace std;
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
int messagerecu =0;

SimpleSerial * serial;
// %Tag(CALLBACK)%
void gps()
{
	int ser;
	char ok;
	string gps_message_uart = "$GPGGA,095455.40,,,,,0,00,99.99,,,,,,*6A";	
std::string delimiter = ",";

//gps_message_uart = serial->readLine();
cout << "i heard normal " << gps_message_uart << endl;

// On parse les résultats
size_t pos = 0;
std::string token;
while ((pos = gps_message_uart.find(delimiter)) != std::string::npos) {
    token = gps_message_uart.substr(0, pos);
    std::cout << token << std::endl;
    gps_message_uart.erase(0, pos + delimiter.length());
}




}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{

  ros::init(argc, argv, "gps");

  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("gps", 1);

ROS_INFO("Init GPS ok");
serial = new SimpleSerial("/dev/ttyAMA0", 9600, 100);
  while (ros::ok())
  {
  usleep(10000);
  //gps();


    ros::spinOnce();

  
  }



  return 0;
}
// %EndTag(FULLTEXT)%

