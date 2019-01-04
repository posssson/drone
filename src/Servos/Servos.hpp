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
#include "drone/Altitude_msg.h"
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
int moteur_deriere_gauche = 6;
int moteur_deriere_droit = 5;
int moteur_devant_droit = 17;
	
// Itération des optimisations
enum IDaltitude{IDbarometre,IDultrason};
int it =0,it_optim=0;
float test = 0;
int recu_init = 0;
float altitude_init[2];
float attitude[6],altitude[2]={0,0};
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
int sens_rotation_z = 1;

void calcul_pid_angle();
void calcul_pid_vitesse();
void calcul_valeur_commande_moteur();
void recuperation_cmd_gaz(int gpio, int level, uint32_t tick);
void recuperation_cmd_tangage(int gpio, int level, uint32_t tick);
void recuperation_cmd_roulis(int gpio, int level, uint32_t tick);
void recuperation_cmd_lacet(int gpio, int level, uint32_t tick);
void recuperation_cmd_arret(int gpio, int level, uint32_t tick);
float saturation(float a,int min,int max);
void recuperation_altitude(const drone::Altitude_msg::ConstPtr& _msg, float _altitude[2]);
void recuperation_capteur(const drone::Capteurs_msg::ConstPtr& _msg, int *recu_init, float _attitude[5]);
void recuperation_clavier(const drone::Clavier_msg::ConstPtr& _msg, float *_kp, float *_ki, float *_kd,float *_kpp,float *_kii,float *_kdd);
void recuperation_optim(const drone::Optim_msg::ConstPtr& _msg, float *_kp,float *_ki, float *_kd,float *_kpp,float *_kdd);
