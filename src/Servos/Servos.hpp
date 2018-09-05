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
#include "drone/Capteurs_msg.h"
#include "drone/Clavier_msg.h"
#include "drone/Optim_msg.h"
#include "drone/Erreur_angle_msg.h"
void recuperation_cmd_gaz(int gpio, int level, uint32_t tick);
void recuperation_cmd_tangage(int gpio, int level, uint32_t tick);
void recuperation_cmd_roulis(int gpio, int level, uint32_t tick);
void recuperation_cmd_lacet(int gpio, int level, uint32_t tick);
void recuperation_cmd_arret(int gpio, int level, uint32_t tick);
float saturation(float a,int min,int max);

uint32_t tick_precedent_arret = 0, temps_haut_arret = 0;
uint32_t tick_precedent_gaz = 0, temps_haut_gaz = 0;
uint32_t tick_precedent_tangage = 0, temps_haut_tangage = 0;
uint32_t tick_precedent_roulis = 0, temps_haut_roulis = 0;
uint32_t tick_precedent_lacet = 0, temps_haut_lacet = 0;
double somme_erreurs_total = 0;
int it =0,it_optim=0;
float test = 0;


void recuperation_cmd_gaz(int gpio, int level, uint32_t tick)
{
	//printf("GPIO %d became %d at %d", gpio, level, tick);
	if (level == 1)
	{
		tick_precedent_gaz = tick;
	}
	else
	{
		temps_haut_gaz = tick - tick_precedent_gaz;
		//printf("GPIO temps_haut_gaz haut = %d", temps_haut_gaz);

	}
}


void recuperation_cmd_tangage(int gpio, int level, uint32_t tick)
{
	//printf("GPIO %d became %d at %d", gpio, level, tick);
	if (level == 1)
	{
		tick_precedent_tangage = tick;
	}
	else
	{
		temps_haut_tangage = tick - tick_precedent_tangage;
		//printf("GPIO temps temps_haut_tangage = %d", temps_haut_tangage);

	}
}

void recuperation_cmd_roulis(int gpio, int level, uint32_t tick)
{
	//printf("GPIO %d became %d at %d", gpio, level, tick);
	if (level == 1)
	{
		tick_precedent_roulis = tick;
	}
	else
	{
		temps_haut_roulis = tick - tick_precedent_roulis;
		//printf("GPIO temps_haut_roulis haut = %d \n", temps_haut_roulis);

	}
}


void recuperation_cmd_lacet(int gpio, int level, uint32_t tick)
{
	//printf("GPIO %d became %d at %d", gpio, level, tick);
	if (level == 1)
	{
		tick_precedent_lacet = tick;
	}
	else
	{
		temps_haut_lacet = tick - tick_precedent_lacet;
		//printf("GPIO temps_haut_lacet haut = %d", temps_haut_lacet);

	}
}


void recuperation_cmd_arret(int gpio, int level, uint32_t tick)
{
	//printf("GPIO %d became %d at %d", gpio, level, tick);
	if (level == 1)
	{
		tick_precedent_arret = tick;
	}
	else
	{
		temps_haut_arret = tick - tick_precedent_arret;
		//printf("GPIO temps haut = %d", temps_haut_arret);

	}
}

float saturation(float a,int min,int max)
{
	//ROS_INFO("Saturation");
	if (a < min)
		a = min;
	if (a > max)
		a = max;
	return a;
}


//float calcul_repartition(float )
void recuperation_capteur(const drone::Capteurs_msg::ConstPtr& _msg, int *recu_init, float _attitude[5])
{
	*recu_init = 1;
	_attitude[0] = _msg->x;
	_attitude[1] = _msg->y;
	_attitude[2] = _msg->z;
	_attitude[3] = _msg->vx;
	_attitude[4] = _msg->vy;
	//ROS_INFO("x y z = %f  %f   %f", _attitude[0], _attitude[1],_attitude[2]);
	//ROS_INFO("vx vy  = %f  %f   ", _attitude[3], _attitude[4]);


}
void recuperation_clavier(const drone::Clavier_msg::ConstPtr& _msg, float *_kp, float *_ki, float *_kd,float *_kpp,float *_kii,float *_kdd)
{
	int reg = 0; // TODO expliciter c'est le tangage 
	*(_kp+reg) += _msg->kp;
	*(_ki+reg) += _msg->ki;
	*(_kd+reg) += _msg->kd;
	*(_kpp+reg) += _msg->kpp;
	*(_kii+reg) += _msg->kii;
	*(_kdd+reg) += _msg->kdd;
	reg = 1; // TODO expliciter c'est le roulis 
	*(_kp+reg) += _msg->kp;
	*(_ki+reg) += _msg->ki;
	*(_kd+reg) += _msg->kd;
	*(_kpp+reg) += _msg->kpp;
	*(_kii+reg) += _msg->kii;
	*(_kdd+reg) += _msg->kdd;

	ROS_INFO("CLAVIER kp ki kd kpp kii kdd = %f  %f   %f   %f %f %f", *(_kp+reg), *(_ki+reg), *(_kd+reg),*(_kpp+reg),*(_kii+reg),*(_kdd+reg));

}



void recuperation_optim(const drone::Optim_msg::ConstPtr& _msg, float *_kp,float *_ki, float *_kd,float *_kpp,float *_kdd)
{
	somme_erreurs_total = 0;
	it_optim =0;
	test = -test;
	int reg = 0;
	*(_kp+reg) = _msg->Kp_vit;
	*(_ki+reg)= _msg->Ki_vit;
	*(_kd+reg) = _msg->Kd_vit;
	*(_kpp+reg) = _msg->Kp;
	*(_kdd+reg) = _msg->Kd;
	reg = 1;
	*(_kp+reg) = _msg->Kp_vit;
	*(_ki+reg)= _msg->Ki_vit;
	*(_kd+reg) = _msg->Kd_vit;
	*(_kpp+reg) = _msg->Kp;
	*(_kdd+reg) = _msg->Kd;
	ROS_INFO("OPTIM kp ki kd kpp = %f  %f   %f  %f %f ", *(_kp+reg),*(_ki+reg), *(_kd+reg),*(_kpp+reg),*(_kdd+reg));
}
