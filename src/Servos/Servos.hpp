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

// Declaration variable pour la récupération des valeurs des consignes commande
uint32_t tick_precedent_arret = 0, temps_haut_arret = 0;
uint32_t tick_precedent_gaz = 0, temps_haut_gaz = 0;
uint32_t tick_precedent_tangage = 0, temps_haut_tangage = 0;
uint32_t tick_precedent_roulis = 0, temps_haut_roulis = 0;
uint32_t tick_precedent_lacet = 0, temps_haut_lacet = 0;

// initialisation variable PID
int roulis = 1, tangage =0 ,lacet = 2;
float gaz = 1100;
float commande_devant_droit = 0, commande_devant_gauche = 0, commande_derierre_gauche = 0, commande_derierre_droit = 0;
float arret = 0;
int raz = 0;
float init_lacet = 0;
int erreur_max = 1;
struct timespec time_actuel,ancien_temps,temps_attente,temps_attente_nanosleep;
int initialisation_lacet = 0;
	
// Somme des erreur envoyées à l'optimisation
double somme_erreurs_total = 0;

// initialisation moteur
int moteur_devant_gauche = 27;
int moteur_deriere_gauche = 17;
int moteur_deriere_droit = 5;
int moteur_devant_droit = 6;
	
// Itération des optimisations
int it =0,it_optim=0;
float test = 0;
int recu_init = 0;
float attitude[5];
float consigne[4]= { 0, 0, 0, 0 };
float clavier[3];
float erreur[3] = { 0,0,0 }, erreur_precedente[3] = { 0,0,0 }, commande[3] = { 0,0,0 }, variation_erreur[3] = { 0,0,0 }, somme_erreurs[3] = {0,0,0 };
float erreur_vit[3] = { 0,0,0 }, erreur_precedente_vit[3] = { 0,0,0 }, commande_vit[3] = { 0,0,0 }, variation_erreur_vit[3] = { 0,0,0 }, somme_erreurs_vit[3] = {0,0,0 };
float Kp[3] = { 3.2 ,3.2, 3}, Ki_default[3] = { 0.000,0.000,0.000 }, Kd_default[3] = { 0.011,0.011,0 };
float Kp_vit[3]={3,3,0},Ki_vit_default[3]={0.00117,0.00117,0.000},Kd_vit_default[3]={0.023,0.023,0.004};
float  Ki[3], Kd[3];
float Ki_vit[3],Kd_vit[3];
int optim = 0;
float erreur_gaz = 0, gaz_actuel = 0;
int premiere_erreur = 1;

void calcul_pid_angle()
{
	// Calcul PID angles roulis/tangage/lacet
	erreur[tangage] = consigne[tangage] - attitude[tangage];
	erreur[roulis] = consigne[roulis] - attitude[roulis];
	erreur[lacet] = consigne[lacet] - (attitude[lacet] - init_lacet);
	somme_erreurs[tangage] +=  erreur[tangage];
	somme_erreurs[tangage] *= raz;
	somme_erreurs[roulis] += erreur[roulis];
	somme_erreurs[roulis] *= raz;
	somme_erreurs[lacet] += erreur[lacet];
	somme_erreurs[lacet] *= raz;
	variation_erreur[tangage] = erreur[tangage] - erreur_precedente[tangage];
	variation_erreur[roulis] = erreur[roulis] - erreur_precedente[roulis];
	variation_erreur[lacet] = erreur[lacet] - erreur_precedente[lacet];
	commande[tangage] =Kp[tangage] * erreur[tangage] + Ki[tangage] * somme_erreurs[tangage] + Kd[tangage] * variation_erreur[tangage];
	commande[roulis] = Kp[roulis] * erreur[roulis] + Ki[roulis] * somme_erreurs[roulis] + Kd[roulis] * variation_erreur[roulis];
	commande[lacet] = Kp[lacet] * erreur[lacet] + Ki[lacet] * somme_erreurs[lacet] + Kd[lacet] * variation_erreur[lacet];
	erreur_precedente[tangage] = erreur[tangage];
	erreur_precedente[roulis] = erreur[roulis];
	erreur_precedente[lacet] = erreur[lacet];
}

void calcul_pid_vitesse()
{
// TODO ne pas laisser 3 et 2
	erreur_vit[tangage] = commande[tangage] - attitude[3];
	erreur_vit[roulis] = commande[roulis] - attitude[4];
	erreur_vit[lacet] = commande[lacet] - 0;
	somme_erreurs_vit[tangage] +=  erreur_vit[tangage];
	somme_erreurs_vit[tangage] *= raz;
	somme_erreurs_vit[roulis] += erreur_vit[roulis];
	somme_erreurs_vit[roulis] *= raz;
	somme_erreurs_vit[lacet] += erreur_vit[lacet];
	somme_erreurs_vit[lacet] *= raz;
	variation_erreur_vit[tangage] = erreur_vit[tangage] - erreur_precedente_vit[tangage];
	variation_erreur_vit[roulis] = erreur_vit[roulis] - erreur_precedente_vit[roulis];
	variation_erreur_vit[lacet] = erreur_vit[lacet] - erreur_precedente_vit[lacet];
	commande_vit[tangage] = Kp_vit[tangage] * erreur_vit[tangage] + Ki_vit[tangage] * somme_erreurs_vit[tangage] + Kd_vit[tangage] * variation_erreur_vit[tangage];
	commande_vit[roulis] = Kp_vit[roulis] * erreur_vit[roulis] + Ki_vit[roulis] * somme_erreurs_vit[roulis] + Kd_vit[roulis] * variation_erreur_vit[roulis];
	commande_vit[lacet] = Kp_vit[lacet] * erreur_vit[lacet] + Ki_vit[lacet] * somme_erreurs_vit[lacet] + Kd_vit[lacet] * variation_erreur_vit[lacet];
	erreur_precedente_vit[tangage] = erreur_vit[tangage];
	erreur_precedente_vit[roulis] = erreur_vit[roulis];
	erreur_precedente_vit[lacet] = erreur_vit[lacet];
}

void calcul_valeur_commande_moteur()
{
	// Ajout de la commande de gaz
			commande_derierre_droit = gaz;
			commande_derierre_gauche = gaz;
			commande_devant_droit = gaz;
			commande_devant_gauche = gaz;

			// Mise en place de la commadne en fonction positif négatif
			if (commande_vit[tangage] < 0)
			{
				commande_devant_droit -= commande_vit[tangage];
				commande_devant_gauche -= commande_vit[tangage];
			}
			if (commande_vit[roulis] > 0)
			{

				commande_derierre_droit += commande_vit[roulis];
				commande_devant_droit += commande_vit[roulis];
			}
			if (commande_vit[tangage] > 0)
			{
				commande_derierre_droit += commande_vit[tangage];
				commande_derierre_gauche += commande_vit[tangage];
			}
			if (commande_vit[roulis] < 0)
			{
				commande_derierre_gauche -= commande_vit[roulis];
				commande_devant_gauche -= commande_vit[roulis];
			}

			commande_devant_gauche -= commande[lacet];
			commande_derierre_droit -= commande[lacet];

			commande_devant_droit += commande[lacet];
			commande_derierre_gauche += commande[lacet];

			// Saturation de la commande
			commande_devant_droit = saturation(commande_devant_droit ,1150,1900);
			commande_devant_gauche = saturation(commande_devant_gauche,1150,1900);
			commande_derierre_droit = saturation(commande_derierre_droit,1150,1900);
			commande_derierre_gauche = saturation(commande_derierre_gauche,1150,1900); 

}



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


