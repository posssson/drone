
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <nlopt.h>
#include <math.h>
#include "drone/Optim_msg.h"
#include "drone/Erreur_angle_msg.h"

double myfunc(unsigned n, const double *x, double *grad, void *my_func_data);
void recuperation_erreur_angle(const drone::Erreur_angle_msg::ConstPtr& _msg, double *erreur);

double erreur_angle = 0;
double meilleur_resultats = 9999999999999;
double meilleur_x[6] = {0,0,0,0,0,0}; /* Kp kp_vit kd_vit */

void recuperation_erreur_angle(const drone::Erreur_angle_msg::ConstPtr& _msg, double *erreur)
{
	
	*erreur = _msg->erreur_angle;
	

	//ROS_INFO("On a recu l'erreur dans callback %f",*erreur);

}



double myfunc(unsigned n, const double *x, double *grad, void *my_func_data)
{
	static ros::NodeHandle n2;
	static ros::Publisher _pub_msg_optim = n2.advertise<drone::Optim_msg>("optimisation", 1);
	ROS_INFO("On est dans l opti");
	drone::Optim_msg msg_optim;
	erreur_angle = 0;
	
	// send 
	if (x[0] > 0)
	{
		//ROS_INFO("%f %f %f %f %f",(float)x[0],(float)x[1],(float)x[2],(float)x[3],(float)x[4]);
		msg_optim.Kp = (float)x[0];
		msg_optim.Ki = (float)x[1];
		msg_optim.Kd = (float)x[2];
		msg_optim.Kp_vit = (float)x[3];
		msg_optim.Ki_vit = (float)x[4];
		msg_optim.Kd_vit = (float)x[5];
		_pub_msg_optim.publish(msg_optim);
	}


	// On attend jusqu'a avoir recu l'erreur correspondatntes aux PID envoyé.
	while(erreur_angle = 0 ) {
		ros::spinOnce();
		usleep(1000);
		//ROS_INFO(" %f",erreur_angle);
	}
	
	
	if (erreur_angle < meilleur_resultats)
	{
		meilleur_resultats = erreur_angle;
		ROS_INFO("meilleur resultats = %f %f %f %f %f %f : Valeur optim %f",(float)x[0],(float)x[1],(float)x[2],(float)x[3],(float)x[4],(float)x[5],meilleur_resultats);
	}
	///ROS_INFO("On a recu l'erreur %f",erreur_angle);

	return ((double)erreur_angle*(double)erreur_angle);
}



double time_in_mill = 0, ancien_temps = 0, temps_proc = 0;
struct timeval tv;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Optim");
	ros::NodeHandle n;
	ros::Publisher _pub_msg_optim = n.advertise<drone::Optim_msg>("optimisation", 1);
	boost::function<void(const drone::Erreur_angle_msg::ConstPtr& msg)> temp = boost::bind(recuperation_erreur_angle, _1, &erreur_angle);
	ros::Subscriber sub = n.subscribe("erreur_angle", 1, temp);

	// SET Optim
	double x[6] = {2,6,0,1,8,0.005}; /* Kp kp_vit kd_vit */
	double pourcentage_bornes = 0.10;
	double lb[6] = {x[0]*0.8,x[1]*0.8 ,x[2]*0.8,x[3]*0.8,x[4]*0.8 ,x[5]*0.8}; /* lower bounds 5%*/
	double ub[6] = {x[0]*1.2 ,x[1]*1.2 ,x[2]*1.2,x[3]*1.2 ,x[4]*1.2 ,x[5]*1.2}; /* upper bounds */
	nlopt_opt opt;
	opt = nlopt_create(NLOPT_GN_CRS2_LM , 6); /* algorithm and dimensionality */
	nlopt_set_lower_bounds(opt, lb);
	nlopt_set_upper_bounds(opt,ub);
	nlopt_set_min_objective(opt, myfunc, NULL);
	nlopt_set_xtol_rel(opt, 0.00001);


	double minf; /* `*`the` `minimum` `objective` `value,` `upon` `return`*` */
	sleep(5);

	// TODO a changer ppour mettre ou non l'optim
	if (true) {
		ROS_INFO("Optimsation lancée");
		if (nlopt_optimize(opt, x, &minf) < 0) {
			ROS_INFO("nlopt failed!\n");
		}
		else {
			ROS_INFO("found minimum at (%f,%f,%f,%f,%f,%f) = %f\n", x[0], x[1],x[2],x[3],x[4],x[5], minf);
		}
	}


	while(1) 
	{
		sleep(1);
	}

	return 0;
}