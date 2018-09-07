#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <time.h>
int main(int argc, char **argv) {
	// Initialization of ROS
	ros::init(argc, argv, "Init");
	ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("initialisation", 0);

sleep(10);

while (ros::ok()) {

	ROS_INFO("Initialisation");  
	std_msgs::String msg;
	std::stringstream ss;
	ss << "Initialisation finie " ;
	msg.data = ss.str();
	chatter_pub.publish(msg);
	//system("raspivid -o ~/Camera/video_vol.h264 -vf -t 1000000000");
        ros::spin();
	}
	return 0;
}
