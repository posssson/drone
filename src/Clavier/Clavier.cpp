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
		msg_clavier.kp = 1;
		msg_clavier.kd = 1;
		msg_clavier.ki = 1;
		msg_clavier.kdd = 1;
		msg_clavier.kpp = 1;
		msg_clavier.kii = 1;
		msg_clavier.ka = 1;
		msg_clavier.kia = 1;
		msg_clavier.kda = 1;
		msg_clavier.kl = 1;
		msg_clavier.kil = 1;
		msg_clavier.kdl = 1;
		char c[10];
		cin >> c;
		cout << "tu as tape : " << c <<endl;
		
		if (!strcmp(c,"a"))
		{msg_clavier.kp = 1.05;
			clear(c);}
		
		if (!strcmp(c, "q"))
		{msg_clavier.kp = 0.95;
			clear(c);}
		
		if (!strcmp(c, "z"))
		{msg_clavier.ki = 1.05;
			clear(c);}
		
		if (!strcmp(c, "s"))
		{msg_clavier.ki = 0.95;
			clear(c);}
		if (!strcmp(c, "e"))
		{
			msg_clavier.kd = 1.05;
			clear(c);}
		if (!strcmp(c, "d"))
		{
			msg_clavier.kd = 0.95;
			clear(c);}
		if (!strcmp(c, "r"))
		{
			msg_clavier.kpp = 1.05;
			clear(c);}
		if (!strcmp(c, "f"))
		{
			msg_clavier.kpp = 0.95;
			clear(c);}
		if (!strcmp(c, "t"))
		{
			msg_clavier.kii = 1.05;
			clear(c);}
		if (!strcmp(c, "g"))
		{
			msg_clavier.kii = 0.95;
			clear(c);}
		if (!strcmp(c, "y"))
		{
			msg_clavier.kdd = 1.05;
			clear(c);}
		if (!strcmp(c, "h"))
		{
			msg_clavier.kdd = 0.95;
			clear(c);}
		
		
		
		
		
		if (!strcmp(c, "u"))
		{
			msg_clavier.kl = 1.05;
			clear(c);}
		if (!strcmp(c, "j"))
		{
			msg_clavier.kl = 0.95;
			clear(c);}
		if (!strcmp(c, "i"))
		{
			msg_clavier.kil = 1.05;
			clear(c);}
		if (!strcmp(c, "k"))
		{
			msg_clavier.kil = 0.95;
			clear(c);}
		if (!strcmp(c, "o"))
		{
			msg_clavier.kdl = 1.05;
			clear(c);}
		if (!strcmp(c, "l"))
		{
			msg_clavier.kdl = 0.95;
			clear(c);}		
		
		if (!strcmp(c, "4"))
		{
			msg_clavier.ka = 1.05;
			clear(c);}
		if (!strcmp(c, "1"))
		{
			msg_clavier.ka = 0.95;
			clear(c);}
		if (!strcmp(c, "5"))
		{
			msg_clavier.kia = 1.05;
			clear(c);}
		if (!strcmp(c, "2"))
		{
			msg_clavier.kia = 0.95;
			clear(c);}
		if (!strcmp(c, "6"))
		{
			msg_clavier.kda = 1.05;
			clear(c);}
		if (!strcmp(c, "3"))
		{
			msg_clavier.kda = 0.95;
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
	c[0] = 0;
	c[1] = 0;

}


// %EndTag(FULLTEXT)%

