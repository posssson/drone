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
#include "drone/Gaz_msg.h"
#include "drone/Clavier_msg.h"
#include "drone/Optim_msg.h"
#include "drone/Erreur_angle_msg.h"
#include "drone/Altitude_msg.h"
#include "drone/Data_msg.h"
#include <thread>
#include "../Common/Realtime.h"
#include <ros/callback_queue.h>

void recuperation_cmd_gaz(int gpio, int level, uint32_t tick);
void recuperation_cmd_tangage(int gpio, int level, uint32_t tick);
void recuperation_cmd_roulis(int gpio, int level, uint32_t tick);
void recuperation_cmd_lacet(int gpio, int level, uint32_t tick);
void recuperation_cmd_arret(int gpio, int level, uint32_t tick);
float saturation(float a,float min,float max);
/*
float Kp[4] = {3.5 ,3.5, 1.3,160.0}, Ki_default[4] = { 6.3,6.3,0.13,2500 }, Kd_default[4] = { 0.034,0.034,0.05,0.005};
float Kp_vit[4] = {2.2,2.2,1,100},Ki_vit_default[4] = {4.1,4.1,0,1000},Kd_vit_default[4] = {0.003,0.003,0.00,0.005};
float limite_ki[4] = {20,20,50,200};
float limite_ki_vit[4] = {200,200,50,200};
*/
// Declaration variable pour la récupération des valeurs des consignes commande
uint32_t tick_precedent_arret = 0, temps_haut_arret = 0;
uint32_t tick_precedent_gaz = 1000, temps_haut_gaz = 1000;
uint32_t tick_precedent_tangage = 0, temps_haut_tangage = 0;
uint32_t tick_precedent_roulis = 0, temps_haut_roulis = 0;
uint32_t tick_precedent_lacet = 0, temps_haut_lacet = 0;

// initialisation variable PID
//int roulis = 1, tangage = 0 ,lacet = 2,altitude = 3;

enum {
	tangage,
	roulis,
	lacet,
	altitude,
};
int gaz = 1000, poussee = 0;
float commande_devant_droit = 0, commande_devant_gauche = 0, commande_derierre_gauche = 0, commande_derierre_droit = 0;
float commande_altitude[2] = {0,0};
float arret = 0;
int raz = 0,raz_altitude = 0, changement_consigne = 1;
float init_lacet = 0;
int erreur_max = 1;
struct timespec time_actuel,ancien_temps,temps_attente,temps_attente_nanosleep,temps_changement_consigne;
int initialisation_lacet = 0, initialisation = 0;
float printTimer = 0;
// Somme des erreur envoyées à l'optimisation
double somme_erreurs_total = 0;

// initialisation moteur
int moteur_devant_gauche = 18;
int moteur_deriere_gauche = 12;
int moteur_deriere_droit = 19; 
int moteur_devant_droit = 13;

// Itération des optimisations
enum {
	IDbarometre,
	IDultrason,
	IDreel,
};

double time_in_mill = 0, temps_proc = 0;
struct timeval tv;

int it = 0,it_optim = 0,stabilisation = 0;
float frequence_pwm = 490.0;
float temps_compensation_frenquence = 0.00002;
float frequenceLoop = 1000;
float time_loop = 1/frequenceLoop;
float securiteAltitude = 0;
float test = 0;
int recu_init = 0;
float accelMag[6],attitude[6],altitude_capteur[3] = {0,0},altitude_precedent[3] = {0,0}; // Altitude baro - ultrason
float vitesse_altitude[2] = {0,0};
float consigne[4] = { 0, 0, 0, 0 };
float clavier[3];
float erreur[4] = { 0,0,0,0 }, erreur_precedente[4] = { 0,0,0,0 }, commande[4] = { 0,0,0 ,0}, variation_erreur[4] = { 0,0,0,0 }, somme_erreurs[4] = {0,0,0,0 };
float erreur_vit[4] = { 0,0,0,0 }, erreur_precedente_vit[4] = { 0,0,0,0 }, commande_vit[4] = { 0,0,0,0 }, variation_erreur_vit[4] = { 0,0,0,0 }, somme_erreurs_vit[4] = {0,0,0,0 };

float Kp[4] = {1.9 ,1.9, 1.3,160.0}, Ki_default[4] = { 5.7,5.7,0.13,1000 }, Kd_default[4] = { 0,0,0.0,0.005};
float Kp_vit[4] = {0.75,0.75,3,38},Ki_vit_default[4] = {4.3,4.3,0.5,0},Kd_vit_default[4] = {0.045,0.045,0.000,0.003};
float limite_ki[4] = {50,50,10,200};
float limite_ki_vit[4] = {200,200,100,150};
float frequence_capteur = 0;
int correctFrequency;
float Ki[4], Kd[4];
float Ki_vit[4],Kd_vit[4];
int optim = 0;
float erreur_gaz = 0, gaz_actuel = 1000;
int premiere_erreur = 1;
int sens_rotation_z = 1;
struct timespec ancien_temps_test,temps_actuel_test;
float temps_test = 0;
int SendData = 0;

void Calcul();
void calcul_pid_angle();
void calcul_pid_vitesse();
void calcul_valeur_commande_moteur();
void recuperation_cmd_gaz(int gpio, int level, uint32_t tick);
void recuperation_cmd_tangage(int gpio, int level, uint32_t tick);
void recuperation_cmd_roulis(int gpio, int level, uint32_t tick);
void recuperation_cmd_lacet(int gpio, int level, uint32_t tick);
void recuperation_cmd_arret(int gpio, int level, uint32_t tick);
void recuperation_altitude(const drone::Altitude_msg::ConstPtr& _msg, float _altitude[2],float _vitesse_altitude[2]);
void recuperation_capteur(const drone::Capteurs_msg::ConstPtr& _msg, int *recu_init, float _attitude[6],float _accelMag[6]);
void recuperation_clavier(const drone::Clavier_msg::ConstPtr& _msg, float *_kp, float *_ki, float *_kd,float *_kpp,float *_kii,float *_kdd);
void recuperation_optim(const drone::Optim_msg::ConstPtr& _msg, float *_kp,float *_ki, float *_kd,float *_kpp,float *_kii, float *_kdd);
void ActualisationPMW();

// Fonctions de communication ROS
boost::function<void(const drone::Capteurs_msg::ConstPtr& msg)> temp = boost::bind(recuperation_capteur, _1, &recu_init, attitude ,accelMag);
boost::function<void(const drone::Altitude_msg::ConstPtr& msg)> temp4 = boost::bind(recuperation_altitude, _1, altitude_capteur,vitesse_altitude);
boost::function<void(const drone::Clavier_msg::ConstPtr& msg)> temp2 = boost::bind(recuperation_clavier, _1,Kp_vit, Ki_vit_default, Kd_vit_default,Kp,Ki_default,Kd_default);
boost::function<void(const drone::Optim_msg::ConstPtr& msg)> temp3 = boost::bind(recuperation_optim, _1,Kp,Ki_default, Kd_default,Kp_vit,Ki_vit_default, Kd_vit_default);
drone::Erreur_angle_msg msg_erreur_angle;
drone::Data_msg msg_data;
drone::Gaz_msg msg_gaz;

ros::Publisher _pub_msg_gaz;
ros::Publisher _pub_msg_data;
ros::Publisher _pub_msg_erreur_angle;
