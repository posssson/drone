#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone/Clavier_msg.h"
#include <signal.h>
#include <unistd.h>

using namespace std;
void clear(char *c);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clavier");
	ros::NodeHandle n;
drone::Clavier_msg msg_clavier;
ros::Publisher _pub_msg_clavier = n.advertise<drone::Clavier_msg>("clavier", 1);
	ROS_INFO("Clavier");


	while (ros::ok())
	{

		usleep(10);
	msg_clavier.kp = 0;
			msg_clavier.kd = 0;
			msg_clavier.ki = 0;
msg_clavier.kdd = 0;msg_clavier.kpp = 0;msg_clavier.kii = 0;
			char c[10];

			cin >> c;

	cout << "tu as tape  : " << c <<endl;
		if (!strcmp(c,"1"))
			{msg_clavier.kp = 0.1;
			clear(c);}
		if (!strcmp(c, "4"))
{
			msg_clavier.kp = -0.1;
clear(c);}
		if (!strcmp(c, "2"))
{
			msg_clavier.ki = 0.001;
clear(c);}
		if (!strcmp(c, "5"))
{
			msg_clavier.ki = -0.001;
clear(c);}
		if (!strcmp(c, "3"))
{
			msg_clavier.kd = 0.001;
clear(c);}
		if (!strcmp(c, "6"))
{
			msg_clavier.kd = -0.001;
clear(c);}
		if (!strcmp(c, "i"))
{
			msg_clavier.kpp = 0.1;
clear(c);}
		if (!strcmp(c, "k"))
			{
			msg_clavier.kpp = -0.1;
		clear(c);}
		if (!strcmp(c, "o"))
		{
			msg_clavier.kii = 0.001;
		clear(c);}
		if (!strcmp(c, "l"))
		{
			msg_clavier.kii = -0.001;
			clear(c);}
		if (!strcmp(c, "p"))
			{
			msg_clavier.kdd = 0.001;
			clear(c);}
		if (!strcmp(c, "m"))
			{
			msg_clavier.kdd = -0.001;
			clear(c);}
			_pub_msg_clavier.publish(msg_clavier);
			
		ros::spinOnce();
		usleep(1); // pour eviter d'avoir une surcharge du processeur en attendant que tous les calculs soient finis 
	}

	// %EndTag(SPIN)%

	return 0;
}

void clear(char *c)
{
   c[0]=0;
c[1] = 0;

}


// %EndTag(FULLTEXT)%

