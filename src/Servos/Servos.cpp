
#include "Servos.hpp"

/*
	float Kp[3] = { 3.2 ,3.2, 3}, Ki_default[3] = { 0.000,0.000,0.000 }, Kd_default[3] = { 0.011,0.011,0 };
	float Kp_vit[3] = {4.5,4.5,0},Ki_vit_default[3] = {0.00117,0.00117,0.000},Kd_vit_default[3] = {0.023,0.023,0.004};
	
*/
using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Servos");
	ros::NodeHandle n;
	Realtime::setup();
	//ros::AsyncSpinner spinner(3);
	//spinner.start();
	//Realtime::setup();
	//Realtime::cpu3();
	//Realtime::realTimeSched();
	_pub_msg_gaz = n.advertise < drone::Gaz_msg > ("gaz", 1);
	_pub_msg_data = n.advertise<drone::Data_msg>("data", 10000);
	_pub_msg_erreur_angle = n.advertise<drone::Erreur_angle_msg>("erreur_angle", 1);
	ros::Subscriber sub3 = n.subscribe("optimisation", 1, temp3);
	ros::Subscriber sub2 = n.subscribe("clavier", 1, temp2);
	ros::Subscriber sub4 = n.subscribe("altitude", 1, temp4);
	ros::Subscriber sub = n.subscribe("capteurs", 1, temp);
	
	
	// Ecriture premiere ligne
	
	// Initialisation des gpio
	gpioTerminate();
	if (gpioInitialise() < 0)
	{
		ROS_INFO("BUG\n");
		gpioTerminate();
		ros::shutdown();
	}
	else
	{
		ROS_INFO(" GPIO OK\n");
	}
	gpioSetMode(6, PI_INPUT);
	gpioSetMode(23, PI_INPUT);
	gpioSetMode(24, PI_INPUT);
	gpioSetMode(25, PI_INPUT);
	gpioSetMode(12, PI_INPUT);
	gpioSetMode(moteur_devant_gauche, PI_OUTPUT);
	gpioSetMode(moteur_deriere_gauche, PI_OUTPUT);
	gpioSetMode(moteur_deriere_droit, PI_OUTPUT);
	gpioSetMode(moteur_devant_droit, PI_OUTPUT);
	
	gpioSetAlertFunc(6, recuperation_cmd_gaz);
	gpioSetAlertFunc(23, recuperation_cmd_tangage);
	gpioSetAlertFunc(24, recuperation_cmd_lacet);
	gpioSetAlertFunc(25, recuperation_cmd_roulis);
	gpioSetAlertFunc(12, recuperation_cmd_arret);
	
	gpioSetPWMrange(moteur_devant_droit,2000);
	gpioSetPWMrange(moteur_devant_gauche, 2000);
	gpioSetPWMrange(moteur_deriere_gauche, 2000);
	gpioSetPWMrange(moteur_deriere_droit, 2000);
	
	gpioSetPWMfrequency(moteur_devant_droit, frequence_pwm);
	gpioSetPWMfrequency(moteur_devant_gauche, frequence_pwm);
	gpioSetPWMfrequency(moteur_deriere_gauche, frequence_pwm);
	gpioSetPWMfrequency(moteur_deriere_droit,frequence_pwm);
	
	gpioPWM(moteur_devant_gauche, 1000);
	gpioPWM(moteur_deriere_gauche, 1000);
	gpioPWM(moteur_deriere_droit, 1000);
	gpioPWM(moteur_devant_droit, 1000);
	// fin initialisation moteur
	
	nanosleep(&temps_attente,&temps_attente_nanosleep);
	
	temps_attente.tv_sec = 0;
	temps_attente.tv_nsec = 1;
	clock_gettime( CLOCK_MONOTONIC, &ancien_temps);
	ROS_INFO("Lancement moteur");
	consigne[roulis] = 0;
	consigne[tangage] = 0;
	consigne[lacet] = 0;
	consigne[altitude] = 0;
	std::thread worker(ActualisationPMW); 
	std::thread worker1(Calcul); 
	/*while (ros::ok())
	{
		
		usleep(1);
		ros::spinOnce();
		}
	*/
	
	//ros::waitForShutdown();
	ros::spin();
	gpioTerminate();
	return 0;
}
void ActualisationPMW()
{	
	sleep(1);
	ros::Rate rate(490); // ROS Rate at 5Hz
	
	while (ros::ok()) {
		clock_gettime( CLOCK_MONOTONIC, &temps_actuel_test);
		temps_test = (temps_actuel_test.tv_sec - ancien_temps_test.tv_sec) + (temps_actuel_test.tv_nsec - ancien_temps_test.tv_nsec)/1000000000.0;	
		clock_gettime( CLOCK_MONOTONIC, &ancien_temps_test);
		
		if (arret == 1 && initialisation_lacet == 1 && gaz < 2000 && gaz > 1100 )
		{
			gpioPWM(moteur_devant_droit, (int)commande_devant_droit); 
			gpioPWM(moteur_devant_gauche, (int)commande_devant_gauche); 
			gpioPWM(moteur_deriere_gauche, (int)commande_derierre_gauche);
			gpioPWM(moteur_deriere_droit, (int)commande_derierre_droit); 
		}
		else // On bloque les moteurs
		{
			gpioPWM(moteur_devant_gauche, 1000);
			gpioPWM(moteur_deriere_gauche, 1000);
			gpioPWM(moteur_deriere_droit, 1000);
			gpioPWM(moteur_devant_droit, 1000);
		}
		rate.sleep();
	}
	ROS_ERROR("FIN PWM");
}

void calcul_pid_angle()
{
	// Calcul PID angles roulis/tangage/lacet
	erreur[tangage] = consigne[tangage] - attitude[tangage];
	erreur[roulis] = consigne[roulis] - attitude[roulis];
	erreur[lacet] = consigne[lacet] - (attitude[lacet] - init_lacet);
	erreur[altitude] = consigne[altitude] - altitude_capteur[IDreel];
	
	if(erreur[lacet] + init_lacet> 180)
	erreur[lacet] = erreur[lacet] - 360;
	else if(erreur[lacet] + init_lacet < -180)
	erreur[lacet] = erreur[lacet] + 360;
	else
	erreur[lacet] = erreur[lacet];//do nothing
	
	somme_erreurs[tangage] += erreur[tangage];
	somme_erreurs[tangage] *= raz;
	somme_erreurs[roulis] += erreur[roulis];
	somme_erreurs[roulis] *= raz;
	somme_erreurs[lacet] += erreur[lacet];
	somme_erreurs[lacet] *= raz;
	somme_erreurs[altitude] += erreur[altitude];
	somme_erreurs[altitude] *= raz;
	
	for (int i = 0 ; i < 4;i++)
	{
		somme_erreurs[i] = saturation(somme_erreurs[i],-limite_ki[i]/Ki[i],limite_ki[i]/Ki[i]);
	}
	
	variation_erreur[tangage] = erreur[tangage] - erreur_precedente[tangage];
	variation_erreur[roulis] = erreur[roulis] - erreur_precedente[roulis];
	variation_erreur[lacet] = erreur[lacet] - erreur_precedente[lacet];
	variation_erreur[altitude] = erreur[altitude] - erreur_precedente[altitude];
	
	commande[tangage] = Kp[tangage] * erreur[tangage] + Ki[tangage] * somme_erreurs[tangage] + Kd[tangage] * variation_erreur[tangage];
	commande[roulis] = Kp[roulis] * erreur[roulis] + Ki[roulis] * somme_erreurs[roulis] + Kd[roulis] * variation_erreur[roulis];
	commande[lacet] = Kp[lacet] * erreur[lacet] + Ki[lacet] * somme_erreurs[lacet] + Kd[lacet] * variation_erreur[lacet];
	commande[altitude] = Kp[altitude] * erreur[altitude] + Ki[altitude] * somme_erreurs[altitude] + Kd[altitude] * variation_erreur[altitude];
	
	for (int i = 0 ; i < 4;i++)
	{
		commande[i] = saturation(commande[i],-limite_ki[i],limite_ki[i]);
	}
	
	erreur_precedente[tangage] = erreur[tangage];
	erreur_precedente[roulis] = erreur[roulis];
	erreur_precedente[lacet] = erreur[lacet];
	erreur_precedente[altitude] = erreur[altitude];
}

void calcul_pid_vitesse()
{
	// TODO ne pas laisser 3 et 2
	commande[lacet] = consigne[lacet];
	erreur_vit[tangage] = commande[tangage] - attitude[3];
	erreur_vit[roulis] = commande[roulis] - attitude[4];
	erreur_vit[lacet] = commande[lacet] - attitude[5];
	
	if(altitude_capteur[IDultrason] > -1 && altitude_capteur[IDultrason] < 2)
	{
		erreur_vit[altitude] = -vitesse_altitude[IDultrason];
	}
	else
	{
		erreur_vit[altitude] = -vitesse_altitude[IDbarometre];
	}
	
	
	somme_erreurs_vit[tangage] += erreur_vit[tangage];
	somme_erreurs_vit[tangage] *= raz;
	somme_erreurs_vit[roulis] += erreur_vit[roulis];
	somme_erreurs_vit[roulis] *= raz;
	somme_erreurs_vit[lacet] += erreur_vit[lacet];
	somme_erreurs_vit[lacet] *= raz;
	somme_erreurs_vit[altitude] += erreur_vit[altitude];
	somme_erreurs_vit[altitude] *= raz;
	
	for (int i = 0 ; i < 4; i++)
	{
		somme_erreurs_vit[i] = saturation(somme_erreurs_vit[i],-limite_ki_vit[i]/Ki_vit[i],limite_ki_vit[i]/Ki_vit[i]);
	}
	
	variation_erreur_vit[tangage] = erreur_vit[tangage] - erreur_precedente_vit[tangage];
	variation_erreur_vit[roulis] = erreur_vit[roulis] - erreur_precedente_vit[roulis];
	variation_erreur_vit[lacet] = erreur_vit[lacet] - erreur_precedente_vit[lacet];
	variation_erreur_vit[altitude] = erreur_vit[altitude] - erreur_precedente_vit[altitude];
	
	commande_vit[tangage] = Kp_vit[tangage] * erreur_vit[tangage] + Ki_vit[tangage] * somme_erreurs_vit[tangage] + Kd_vit[tangage] * variation_erreur_vit[tangage];
	commande_vit[roulis] = Kp_vit[roulis] * erreur_vit[roulis] + Ki_vit[roulis] * somme_erreurs_vit[roulis] + Kd_vit[roulis] * variation_erreur_vit[roulis];
	commande_vit[lacet] = Kp_vit[lacet] * erreur_vit[lacet] + Ki_vit[lacet] * somme_erreurs_vit[lacet] + Kd_vit[lacet] * variation_erreur_vit[lacet];
	commande_vit[altitude] = Kp_vit[altitude] * erreur_vit[altitude] + Ki_vit[altitude] * somme_erreurs_vit[altitude] + Kd_vit[altitude] * variation_erreur_vit[altitude];
	
	for (int i = 0 ; i < 4 ; i++)
	{
		commande_vit[i] = saturation(commande_vit[i],-limite_ki_vit[i],limite_ki_vit[i]);
	}
	
	erreur_precedente_vit[tangage] = erreur_vit[tangage];
	erreur_precedente_vit[roulis] = erreur_vit[roulis];
	erreur_precedente_vit[lacet] = erreur_vit[lacet];
	erreur_precedente_vit[altitude] = erreur_vit[altitude];
	
}

void calcul_valeur_commande_moteur()
{
	//commande_vit[lacet] = 0;
	//commande_vit[tangage] = 0;
	//commande_vit[roulis] = 0;
	commande[altitude] = 0;
	commande_vit[altitude] = 0;
	poussee = gaz + commande[altitude] + commande_vit[altitude];
	poussee = saturation(poussee,1050,1700);	

	// Ajout de la commande de gaz
	commande_derierre_droit = poussee;
	commande_derierre_gauche = poussee;
	commande_devant_droit = poussee;
	commande_devant_gauche = poussee;	
	
	// Mise en place de la commadne en fonction positif négatif
	commande_devant_droit -= commande_vit[tangage];
	commande_derierre_gauche += commande_vit[tangage];
	commande_devant_gauche -= commande_vit[tangage];
	commande_derierre_droit += commande_vit[tangage];

	commande_devant_gauche += commande_vit[roulis];
	commande_devant_droit -= commande_vit[roulis];
	commande_derierre_gauche += commande_vit[roulis];
	commande_derierre_droit -= commande_vit[roulis];

	//ROS_INFO("commande[lacet] = %f \n", commande[lacet]);

	commande_devant_gauche += commande_vit[lacet];
	commande_derierre_droit += commande_vit[lacet];

	commande_devant_droit -= commande_vit[lacet];
	commande_derierre_gauche -= commande_vit[lacet];

	// Saturation de la commande
	commande_devant_droit = saturation(commande_devant_droit ,1050,1950);
	commande_devant_gauche = saturation(commande_devant_gauche,1050,1950);
	commande_derierre_droit = saturation(commande_derierre_droit,1050,1950);
	commande_derierre_gauche = saturation(commande_derierre_gauche,1050,1950);
}

void recuperation_cmd_gaz(int gpio, int level, uint32_t tick)
{
	//ROS_INFO("GPIO %d became %d at %d", gpio, level, tick);
	if (level == 1)
	{
		tick_precedent_gaz = tick;
	}
	else
	{
		temps_haut_gaz = tick - tick_precedent_gaz;
		//ROS_INFO("GPIO temps_haut_gaz haut = %d \n", temps_haut_gaz);

	}
}

void recuperation_cmd_tangage(int gpio, int level, uint32_t tick)
{
	//ROS_INFO("GPIO %d became %d at %d", gpio, level, tick);
	if (level == 1)
	{
		tick_precedent_tangage = tick;
	}
	else
	{
		temps_haut_tangage = tick - tick_precedent_tangage;
		//ROS_INFO("GPIO temps temps_haut_tangage = %d", temps_haut_tangage);

	}
}

void recuperation_cmd_roulis(int gpio, int level, uint32_t tick)
{
	//ROS_INFO("GPIO %d became %d at %d", gpio, level, tick);
	if (level == 1)
	{
		tick_precedent_roulis = tick;
	}
	else
	{
		temps_haut_roulis = tick - tick_precedent_roulis;
		//ROS_INFO("GPIO temps_haut_roulis haut = %d \n", temps_haut_roulis);

	}
}

void recuperation_cmd_lacet(int gpio, int level, uint32_t tick)
{
	//ROS_INFO("GPIO %d became %d at %d", gpio, level, tick);
	if (level == 1)
	{
		tick_precedent_lacet = tick;
	}
	else
	{
		temps_haut_lacet = tick - tick_precedent_lacet;
		//ROS_INFO("GPIO temps_haut_lacet haut = %d \n", temps_haut_lacet);

	}
}

void recuperation_cmd_arret(int gpio, int level, uint32_t tick)
{
	//ROS_INFO("GPIO %d became %d at %d", gpio, level, tick);
	if (level == 1)
	{
		tick_precedent_arret = tick;
	}
	else
	{
		temps_haut_arret = tick - tick_precedent_arret;
		//ROS_INFO("GPIO temps haut = %d", temps_haut_arret);

	}
}

float saturation(float a,float min,float max)
{
	//ROS_INFO("Saturation");
	if (a < min)
	a = min;
	if (a > max)
	a = max;
	return a;
}


void recuperation_altitude(const drone::Altitude_msg::ConstPtr& _msg, float _altitude[2],float _vitesse_altitude[2])
{
	_altitude[0] = _msg->altitude_baro;
	_altitude[1] = _msg->altitude_ultrason;
	_vitesse_altitude[0] = _msg->vitesse_altitude_baro;
	_vitesse_altitude[1] = _msg->vitesse_altitude_ultrason;

}

void recuperation_capteur(const drone::Capteurs_msg::ConstPtr& _msg, int *recu_init, float _attitude[6], float _accelMag[6])
{
	*recu_init = 1;
	_attitude[0] = _msg->roll;
	_attitude[1] = _msg->pitch;
	_attitude[2] = _msg->yaw;
	_attitude[3] = _msg->gx;
	_attitude[4] = _msg->gy;
	_attitude[5] = _msg->gz;
	
	_accelMag[0] = _msg->ax;
	_accelMag[1] = _msg->ay;
	_accelMag[2] = _msg->az;
	_accelMag[3] = _msg->mx;
	_accelMag[4] = _msg->my;
	_accelMag[5] = _msg->mz;
	frequence_capteur = _msg->frequence_capteur;
	//Calcul();
	//ROS_INFO("x y z = %f %f %f", _attitude[0], _attitude[1],_attitude[2]);
	//ROS_INFO("vx vy = %f %f ", _attitude[3], _attitude[4]);
}

void recuperation_clavier(const drone::Clavier_msg::ConstPtr& _msg, float *_kp, float *_ki, float *_kd,float *_kpp,float *_kii,float *_kdd)
{
	int reg = 0; // TODO expliciter c'est le tangage
	*(_kp+reg) *= _msg->kp;
	*(_ki+reg) *= _msg->ki;
	*(_kd+reg) *= _msg->kd;
	*(_kpp+reg) *= _msg->kpp;
	*(_kii+reg) *= _msg->kii;
	*(_kdd+reg) *= _msg->kdd;
	reg = 1; // TODO expliciter c'est le roulis
	*(_kp+reg) *= _msg->kp;
	*(_ki+reg) *= _msg->ki;
	*(_kd+reg) *= _msg->kd;
	*(_kpp+reg) *= _msg->kpp;
	*(_kii+reg) *= _msg->kii;
	*(_kdd+reg) *= _msg->kdd;
	reg = 2; // TODO expliciter c'est le lacet
	*(_kpp+reg) *= _msg->kl;
	*(_kii+reg) *= _msg->kil;
	*(_kdd+reg) *= _msg->kdl;
	reg = 3; // TODO expliciter c'est l'altitude
	*(_kp+reg) *= _msg->ka;
	*(_ki+reg) *= _msg->kia;
	*(_kd+reg) *= _msg->kda;
	
	it_optim = 0;
	//changement_consigne = 1;
	//consigne[roulis] *= 1;
	clock_gettime( CLOCK_MONOTONIC, &temps_changement_consigne);
	ROS_INFO("CLAVIER ROULIS TANGAGE kp ki kd = %f %f %f",*(_kpp+1),*(_kii+1),*(_kdd+1));
	ROS_INFO("CLAVIER ROULIS TANGAGE kp_vit ki_vit kd_vit = %f %f %f", *(_kp+1), *(_ki+1), *(_kd+1));
	ROS_INFO("CLAVIER LACET kp ki kd = %f %f %f",*(_kpp+2),*(_kii+2),*(_kdd+2));
	ROS_INFO("CLAVIER ALTITUE kp ki kd = %f %f %f",*(_kp+3),*(_ki+3),*(_kd+3));
}

void recuperation_optim(const drone::Optim_msg::ConstPtr& _msg, float *_kp,float *_ki, float *_kd,float *_kpp,float *_kii, float *_kdd)
{
	somme_erreurs_total = 0;
	it_optim = 0;
	changement_consigne = 1;
	consigne[roulis] *= -1;
	int reg = 0;
	*(_kp+reg) = _msg->Kp;
	*(_ki+reg) = _msg->Ki;
	*(_kd+reg) = _msg->Kd;
	reg = 1;
	*(_kp+reg) = _msg->Kp;
	*(_ki+reg) = _msg->Ki;
	*(_kd+reg) = _msg->Kd;
	
	reg = 0;
	*(_kpp+reg) = _msg->Kp_vit;
	*(_kii+reg) = _msg->Ki_vit;
	*(_kdd+reg) = _msg->Kd_vit;
	reg = 1;
	*(_kpp+reg) = _msg->Kp_vit;
	*(_kii+reg) = _msg->Ki_vit;
	*(_kdd+reg) = _msg->Kd_vit;
	

	ROS_INFO("OPTIM kp ki kd kpp = %f %f %f ",*(_kp+reg),*(_ki+reg), *(_kd+reg));
	ROS_INFO("OPTIM kp ki kd kpp = %f %f %f ",*(_kpp+reg),*(_kii+reg), *(_kdd+reg));
}

void Calcul()
{
	Realtime::cpu3();
	Realtime::realTimeSched();
	ros::Rate rate(frequenceLoop); // ROS Rate at 100Hz
	while(!recu_init)
	{
		usleep(100);
	}
	ROS_INFO("Pret pour le signal de départ");
	while (ros::ok()) {
		if (initialisation_lacet == 0 && fabs(attitude[tangage]) < 5 && fabs(attitude[roulis]) < 5 && temps_haut_gaz < 1080 && temps_haut_gaz > 1040 && recu_init == 1 && temps_haut_tangage < 1250 && temps_haut_roulis < 1250 && temps_haut_lacet > 1750 )
		{
			ROS_INFO("Tout est OK Initialisation OK");
			ROS_INFO("arret gaz = %d\n", temps_haut_gaz);
			
			init_lacet = attitude[lacet];
			initialisation_lacet = 1;
			initialisation = 1;
		}
		// Initialisation vrai quand tout est initialisé
		if (initialisation)
		{
			// Boucle permettant d'avoir un temps de cycle constant
			clock_gettime( CLOCK_MONOTONIC, &time_actuel);
			temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;
			if (temps_proc < time_loop)
			{
				correctFrequency = (int)((time_loop - temps_proc)*1000000);
				if (correctFrequency > 120)
				{
					correctFrequency -= 40;
					usleep(correctFrequency);
				}
				while (temps_proc < time_loop)
				{
					clock_gettime( CLOCK_MONOTONIC, &time_actuel);
					temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;	
				}
				clock_gettime( CLOCK_MONOTONIC, &time_actuel);
				temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;	
			}
			ancien_temps = time_actuel;
			// Ajustement PID en fonction du temps
			for (int i = 0 ; i < 4 ; i++) {
				Ki[i] = Ki_default[i]*(float)temps_proc;
				Ki_vit[i] = Ki_vit_default[i]*(float)temps_proc;
				Kd[i] = Kd_default[i]/(float)temps_proc;
				Kd_vit[i] = Kd_vit_default[i]/(float)temps_proc;
			}
			// Commande GAZ
			if (temps_haut_gaz > 1000 && temps_haut_gaz < 1850)
			{
				gaz = temps_haut_gaz;
				gaz = saturation(gaz,1000,1700);
			}
			else
			{
				gaz -= 100 / frequenceLoop; // Si probleme de reception gaz on reduit le gaz de 100 par seconde.
				gaz = saturation(gaz,1000,1700);
			}
			msg_gaz.gaz = gaz;
			_pub_msg_gaz.publish(msg_gaz); 
			//
			

			
			// Calcul PID Commande roulis/tangage
			if (true)
			{
				if (temps_haut_roulis > 1200 && temps_haut_roulis < 1800)
				{
					consigne[roulis] = -((float)temps_haut_roulis- 1500.0)/10.0;
					consigne[roulis] = (fabs(consigne[roulis])*consigne[roulis])/40;
				}
				else
				{
					consigne[roulis] = 0;
					temps_haut_roulis = 0;
				}
				if (temps_haut_tangage > 1200 && temps_haut_tangage < 1800)
				{
					consigne[tangage] = ((float)temps_haut_tangage- 1500.0)/10.0;
					consigne[tangage] = (fabs(consigne[tangage])*consigne[tangage])/40;
				}
				else
				{
					consigne[tangage] = 0;
					temps_haut_tangage = 0;
				}
				if (temps_haut_lacet > 1200 && temps_haut_lacet < 1800)
				{
					consigne[lacet] = ((float)temps_haut_lacet - 1500.0)/10.0;
					consigne[lacet] = (fabs(consigne[lacet])*consigne[lacet])/20;
				}
				else
				{
					consigne[lacet] = 0;
					temps_haut_lacet = 0;
				}
				
			}
			else 
			{
				if (gaz > 1100)
				{
					//ROS_INFO("changement_consigne %d",changement_consigne);
					consigne[roulis] += 5.0/4000.0*(float)changement_consigne;
					if (consigne[roulis] > 10)
					{
						consigne[roulis] = 10;
						changement_consigne *= -1;
					}
					if (consigne[roulis] < -10)
					{
						consigne[roulis] = -10;
						changement_consigne *= -1;
						
					}
					//ROS_INFO("ROulis %f",consigne[roulis]);
				}
				else
				{
					consigne[roulis] = 0;
				}
				consigne[tangage] = 0;
				consigne[lacet] = 0;
			}
			
			// On choisit si on prend le capteur ultrason ou le barometre en fonction de l'altitude
			if (altitude_capteur[IDultrason] > -1 && altitude_capteur[IDultrason] < 2)
			{
				altitude_capteur[IDreel] = altitude_capteur[IDultrason];
			}
			else
			{
				altitude_capteur[IDreel] = altitude_capteur[IDbarometre];
			}
			
			if (altitude_capteur[IDreel] <0.01)
			{
				altitude_capteur[IDreel] = 0;
			}
			
			
			// CONSIGNE ALTITUDE
			if ( gaz < 1380 && altitude_capteur[IDreel] > 0.05)
			{
				stabilisation = -1;
				consigne[altitude] = altitude_capteur[IDreel] - 0.01;
			}
			if ( gaz < 1380 && altitude_capteur[IDreel] < 0.05 && altitude_capteur[IDreel] > -1)
			{
				stabilisation = -1;
				consigne[altitude] = altitude_capteur[IDreel];
			}
			
			if ( gaz > 1380 && gaz < 1480 && stabilisation != 0 && altitude_capteur[IDreel] > 0.05)
			{
				stabilisation = 0;
				consigne[altitude] = altitude_capteur[IDreel];
			}
			if ( gaz > 1480)
			{
				stabilisation = 1;
				consigne[altitude] = altitude_capteur[IDreel] + 0.01;		
			}
			
			// SECURITE POUR TEST INTERIEUR
			if ( altitude_capteur[IDreel] > 0.2 )
			{
				//ROS_ERROR(" TU VOLES TROP HAUT ");
				securiteAltitude -= 100 / frequenceLoop; // Si probleme de reception gaz on reduit le gaz de 100 par seconde. 
				//gaz = securiteAltitude;
			}
			else
			{
				securiteAltitude = gaz;
			}
			
			
			// RAZ permet de mettre le Ki ou pas.
			if ( gaz < 1300 )
			{
				raz = 0;
				init_lacet = attitude[lacet];
				consigne[lacet] = 0;
			}
			else
			{
				raz = 1;
			}
			
			if (altitude_capteur[IDreel] < 0.01)
			{
				//raz = 0;
			}

			
			
			consigne[altitude] = saturation(consigne[altitude],0,0.2);
			
			// Calcul PID Angles
			calcul_pid_angle();
			// Calcul PID vitesse
			calcul_pid_vitesse();
			calcul_valeur_commande_moteur();
			
			
			
			// Sécurité sur les angles pour les tests 
			if (fabs( attitude[tangage]) > 40 || fabs( attitude[roulis])>40 )
			{
				//ROS_INFO("SECURITE : Tangage roulis gaz = %f %f %d\n",fabs( attitude[tangage]),fabs( attitude[roulis]),gaz);
				gpioPWM(moteur_devant_gauche, 1000);
				gpioPWM(moteur_deriere_gauche, 1000);
				gpioPWM(moteur_deriere_droit, 1000);
				gpioPWM(moteur_devant_droit, 1000);
				arret = 0;
				ROS_ERROR("SECURITE\n");
				while(gaz >1080)
				{
					usleep(100);
					gaz = temps_haut_gaz;
					
				}
				ROS_INFO("FIN SECURITE\n");
			}
			else
			{
				
				arret = 1;
			}
			optim = 0; // todo changer pour lancer optim
			printTimer += temps_proc;
			if (printTimer > 0.5 && true)
			{
				printTimer = 0;
				//ROS_INFO("frequence servos = %f\n", 1/temps_proc);
				//ROS_INFO("gaz = %d\n", gaz);
				/*	ROS_INFO("commande[altitude] = %f\n", commande[altitude]);
					ROS_INFO("commande_vit[altitude] = %f\n", commande_vit[altitude]);
					ROS_INFO("erreur_vit[altitude] = %f\n", erreur_vit[altitude]);*/
				//ROS_INFO("erreur[lacet]] = %f\n", erreur[lacet]);
				//ROS_INFO("erreur_vit[lacet]] = %f\n", erreur_vit[lacet]);
				//ROS_INFO("frequence = %f\n", 1/temps_proc);
				//ROS_INFO("roulis  = %6.6f\n", attitude[roulis]);
				//ROS_INFO("temps_test  = %f\n", temps_test);
				//ROS_INFO("consigne[lacet] = %f\n", (float)consigne[lacet]);
			}
			if (gaz > 1100)
			{
				for (int i = 0 ; i < 4 ; i++)
				{
					msg_data.Kp[i] = Kp[i];
					msg_data.Ki[i] = Ki[i];
					msg_data.Kd[i] = Kd[i];
					msg_data.Kp_vit[i] = Kp_vit[i];
					msg_data.Ki_vit[i] = Ki_vit[i];
					msg_data.Kd_vit[i] = Kd_vit[i];
					msg_data.consigne[i] = consigne[i];
					msg_data.erreur[i] = erreur[i];
					msg_data.somme_erreurs[i] = somme_erreurs[i];
					msg_data.variation_erreur[i] = variation_erreur[i];
					msg_data.erreur_vit[i] = erreur_vit[i];
					msg_data.somme_erreurs_vit[i] = somme_erreurs_vit[i];
					msg_data.variation_erreur_vit[i] = variation_erreur_vit[i];
				}
				for (int i = 0 ; i < 3 ; i++)
				{
					msg_data.altitude_capteur[i] = altitude_capteur[i];
				}
				for (int i = 0 ; i < 6 ; i++)
				{
					msg_data.accelMag[i] = accelMag[i];
					msg_data.attitude[i] = attitude[i];
				}
				for (int i = 0 ; i < 2 ; i++)
				{
					msg_data.altitude_vit[i] = vitesse_altitude[i];
				}
				
				msg_data.temps_proc = temps_proc;
				msg_data.commande_devant_droit = commande_devant_droit;
				msg_data.commande_devant_gauche = commande_devant_gauche;
				msg_data.commande_derierre_droit = commande_derierre_droit;
				msg_data.commande_derierre_gauche = commande_derierre_gauche;
				msg_data.init_lacet = init_lacet;
				msg_data.gaz = gaz;
				msg_data.frequence_capteur = frequence_capteur;
				_pub_msg_data.publish(msg_data);
			}
			
			if (gaz > 1100 && gaz < 2000 && changement_consigne) {
				it_optim ++;
				
				//somme_erreurs_total += ((fabs(erreur_vit[roulis]) + fabs(erreur_vit[tangage]))/temps_proc)/10; 
				//somme_erreurs_total += (fabs(erreur[roulis]) + fabs(erreur[tangage]))/temps_proc; 
				//somme_erreurs_total += ((fabs(erreur_vit[roulis]))/temps_proc)/10; 
				somme_erreurs_total += (fabs(erreur[roulis])); 
				/*float precision = fabs( attitude[tangage]/consigne[tangage]);
				float temps_precision = 0;
				if ( precision<0.95 || precision>1.05 )
				{
					temps_precision = (time_actuel.tv_sec - temps_changement_consigne.tv_sec) + (time_actuel.tv_nsec - temps_changement_consigne.tv_nsec)/1000000000.0;
			}*/
				
				
				if (it_optim > 16000 && premiere_erreur == 1)
				{
					// On evite de prendre les premieres valeurs
					ROS_INFO("Optim lancée Servos");
					premiere_erreur = 0;
					it_optim = 0;
				}
				if (it_optim > 12000 && premiere_erreur == 0 && changement_consigne)
				{	
					somme_erreurs_total /= it_optim;
					msg_erreur_angle.erreur_angle = somme_erreurs_total;
					ROS_INFO("L'erreur moyenne en angle est de %f",somme_erreurs_total);
					//ROS_INFO("Le temps à 5 pour cent est de %f",temps_precision);
					_pub_msg_erreur_angle.publish(msg_erreur_angle);
					somme_erreurs_total = 0;
					it_optim = 0;
					//changement_consigne = 0;
				}		
			}
			recu_init = 0;
			rate.sleep();
		}
		else
		{
			usleep(100);
			clock_gettime( CLOCK_MONOTONIC, &time_actuel);
			ancien_temps = time_actuel;
		}
	}
	
}


