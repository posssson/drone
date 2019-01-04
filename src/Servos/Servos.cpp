
#include "Servos.hpp"

/*
	float Kp[3] = { 3.2 ,3.2, 3}, Ki_default[3] = { 0.000,0.000,0.000 }, Kd_default[3] = { 0.011,0.011,0 };
	float Kp_vit[3]={4.5,4.5,0},Ki_vit_default[3]={0.00117,0.00117,0.000},Kd_vit_default[3]={0.023,0.023,0.004};

 */
using namespace std;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "Servos");
	ros::NodeHandle n;

	double time_in_mill = 0, temps_proc = 0;

	struct timeval  tv;

	// Fonctions de communication ROS
	boost::function<void(const drone::Capteurs_msg::ConstPtr& msg)> temp = boost::bind(recuperation_capteur, _1, &recu_init, attitude);
	ros::Subscriber sub = n.subscribe("capteurs", 1, temp);

	boost::function<void(const drone::Altitude_msg::ConstPtr& msg)> temp4 = boost::bind(recuperation_altitude, _1, altitude);
	ros::Subscriber sub4 = n.subscribe("altitude", 1, temp4);

	boost::function<void(const drone::Clavier_msg::ConstPtr& msg)> temp2 = boost::bind(recuperation_clavier, _1,Kp_vit, Ki_vit_default, Kd_vit_default,Kp,Ki_default,Kd_default);
	ros::Subscriber sub2 = n.subscribe("clavier", 1, temp2);

	boost::function<void(const drone::Optim_msg::ConstPtr& msg)> temp3 = boost::bind(recuperation_optim, _1,Kp_vit,Ki_vit_default, Kd_vit_default,Kp,Kd_default);
	ros::Subscriber sub3 = n.subscribe("optimisation", 1, temp3);

	drone::Erreur_angle_msg msg_erreur_angle;
	ros::Publisher _pub_msg_erreur_angle = n.advertise<drone::Erreur_angle_msg>("erreur_angle", 1);

	// Ouverture d'un fichier pour stocker les données
	ofstream fichier("/home/pi/drone_ws/src/drone/src/donnees.txt", ios::out | ios::trunc); 

	// Initialisation des gpio
	if (gpioInitialise() < 0)
	{
		printf("BUG\n");
		gpioTerminate();
		ros::shutdown();
	}
	else
	{
		printf(" GPIO OK\n");
	}

	gpioSetAlertFunc(13, recuperation_cmd_gaz);
	gpioSetAlertFunc(23, recuperation_cmd_tangage);
	gpioSetAlertFunc(24, recuperation_cmd_lacet);
	gpioSetAlertFunc(26, recuperation_cmd_roulis);
	gpioSetAlertFunc(12, recuperation_cmd_arret);

	gpioSetMode(moteur_devant_gauche, PI_OUTPUT);
	gpioSetMode(moteur_deriere_gauche, PI_OUTPUT);
	gpioSetMode(moteur_deriere_droit, PI_OUTPUT);
	gpioSetMode(moteur_devant_droit, PI_OUTPUT);

	gpioSetPWMrange(moteur_devant_droit,2000);
	gpioSetPWMrange(moteur_devant_gauche, 2000);
	gpioSetPWMrange(moteur_deriere_gauche, 2000);
	gpioSetPWMrange(moteur_deriere_droit, 2000);

	gpioSetPWMfrequency(moteur_devant_droit, 500);
	gpioSetPWMfrequency(moteur_devant_gauche, 500);
	gpioSetPWMfrequency(moteur_deriere_gauche, 500);
	gpioSetPWMfrequency(moteur_deriere_droit,500);


	gpioPWM(moteur_devant_gauche, 1000);
	gpioPWM(moteur_deriere_gauche, 1000);
	gpioPWM(moteur_deriere_droit, 1000);
	gpioPWM(moteur_devant_droit, 1000);
	// fin initialisation moteur

	nanosleep(&temps_attente,&temps_attente_nanosleep);

	temps_attente.tv_sec = 0;
	temps_attente.tv_nsec = 10;
	clock_gettime( CLOCK_REALTIME, &ancien_temps);
	ROS_INFO("Lancement moteur");
	while (ros::ok())
	{

		// Initialisation vrai quand tout est initialisé
		if (recu_init == 1)
		{
			if (initialisation_lacet == 0 && abs(attitude[tangage])<5 && abs(attitude[roulis])<5)
			{
				ROS_INFO("Tout est OK");

				init_lacet = attitude[lacet];
				initialisation_lacet = 1;
			}
			// Boucle permettant d'avoir un temps de cycle de 0.002 (500 Hz)
			clock_gettime( CLOCK_REALTIME, &time_actuel);
			temps_proc =(time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;
			while (temps_proc < 0.002)
			{
				clock_gettime( CLOCK_REALTIME, &time_actuel);
				temps_proc =(time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;
				nanosleep(&temps_attente,&temps_attente_nanosleep);
			}
			ancien_temps = time_actuel;
			// ajustement PID en fonction du temps
			for (int i=0;i<3;i++) {
				Ki[i] = Ki_default[i]*(float)temps_proc;
				Ki_vit[i] = Ki_vit_default[i]*(float)temps_proc;
				Kd[i] = Kd_default[i]/(float)temps_proc;
				Kd_vit[i] = Kd_vit_default[i]/(float)temps_proc;
			}

			// Calcul PID Commande GAZ //TODO
			gaz += (-gaz_actuel + temps_haut_gaz)*0.15;
			//ROS_INFO("GAZ = %f",gaz);
			gaz_actuel = gaz;

			// Calcul PID Commande roulis/tangage
			if (fabs(((float)temps_haut_roulis- 1500.0))>10)
			{consigne[roulis] =((float)temps_haut_roulis- 1500.0)/10.0;}
			if (fabs(((float)temps_haut_tangage- 1500.0))>10)
			{consigne[tangage] =-((float)temps_haut_tangage - 1500.0)/10.0;}
			if (fabs(((float)temps_haut_lacet- 1500.0))>10)
			{
				consigne[lacet] +=((float)temps_haut_lacet - 1500.0)*temps_proc*sens_rotation_z/10.0;
			}

			// Mise à zéro si test sans manette
			consigne[tangage] = 0;
			consigne[roulis] = 0;

			// RAZ permet de mettre le Ki ou pas.
			if (gaz < 1100)
			{
				raz = 0;
				init_lacet = attitude[lacet];
				consigne[lacet] = 0;
				altitude_init[IDbarometre] = altitude[IDbarometre];
				altitude_init[IDultrason] = altitude[IDultrason];
			}
			else
			{
				raz = 1;
			}


			// Compute altitude commande
			// Entre 1100 et 1400 on monte de 1400 a 1500 on garde l'altitude de 1500 a 1800 on monte
			if (gaz<1400 && gaz>1100)
			{

			}





			// Limite de gaz  // TODO pourquoi c'est la???
			if (gaz > 1900)
				gaz = 1900;

			// Calcul PID Angles
			calcul_pid_angle();

			// Saturation de la commande en angle.
			commande[tangage] = saturation(commande[tangage],-35,35);
			commande[roulis] = saturation(commande[roulis],-35,35);

			// Calcul PID vitesse
			calcul_pid_vitesse();

			// Saturation commadne tangage et roulis
			commande_vit[tangage] = saturation(commande_vit[tangage],-250,250);
			commande_vit[roulis] = saturation(commande_vit[roulis],-250,250);

			calcul_valeur_commande_moteur();

			// security gaz can be set
			if (arret == 0 && gaz <1100)
			{
				arret = 1;
			}

			// Sécurité sur les angles pour les tests 
			if ((abs( attitude[tangage]) > 25 || abs( attitude[roulis])>25) && arret == 1)
			{
				arret = 0;

			}

			if (arret == 1 && gaz >1100 && initialisation_lacet==1)
			{
				optim = 0; // todo changer pour lancer optim
				gpioPWM(moteur_devant_droit, commande_devant_droit);  // OK
				gpioPWM(moteur_devant_gauche, commande_devant_gauche);  // Moyenement OK
				gpioPWM(moteur_deriere_droit, commande_derierre_droit);   // OK
				gpioPWM(moteur_deriere_gauche, commande_derierre_gauche);
				// Inscription dans un fichier les valeurs
				fichier << " " << attitude[tangage] << " " << attitude[roulis] << " "<<attitude[4] << " " << attitude[3]  << " " << commande[tangage]<<" " << commande[roulis] << "  " << commande_vit[tangage] << " " << commande_vit[roulis] << endl;
			}

			else // On bloque les moteurs
			{
				optim = 1;
				gpioPWM(moteur_devant_gauche, 1000);
				gpioPWM(moteur_deriere_gauche, 1000);
				gpioPWM(moteur_deriere_droit, 1000);
				gpioPWM(moteur_devant_droit, 1000);
			}

			if (optim == 1 && gaz >1100) {
				//it_optim ++;
				somme_erreurs_total += ((abs(erreur_vit[roulis]) + abs(erreur_vit[tangage]))/temps_proc)/10;  
				somme_erreurs_total += (abs(erreur[roulis]) + abs(erreur[tangage]))/temps_proc;  
				if (it_optim > 1000 && premiere_erreur == 1)
				{
					// On evite de prendre les premieres valeurs
					premiere_erreur = 0;
					it_optim = 0;
				}
				if (it_optim > 2500 && premiere_erreur == 0)
				{	
					msg_erreur_angle.erreur_angle = somme_erreurs_total;
					_pub_msg_erreur_angle.publish(msg_erreur_angle);
					somme_erreurs_total = 0;
					it_optim =0;
				}

			}


		}
		else
		{
			usleep(100);
		}

		ros::spinOnce();

	}


	return 0;
}


void calcul_pid_angle()
{
	// Calcul PID angles roulis/tangage/lacet
	erreur[tangage] = consigne[tangage] - attitude[tangage];
	erreur[roulis] = consigne[roulis] - attitude[roulis];
	erreur[lacet] = consigne[lacet] - attitude[lacet] + init_lacet;
//	ROS_INFO(" CONSIG LACET = %f",consigne[lacet]);
//	ROS_INFO(" ATTITU LACET = %f",attitude[lacet]);
	/*ROS_INFO(" temps_haut_lacet = %f",(float)temps_haut_lacet);
	ROS_INFO(" temps_haut_rouli = %f",(float)temps_haut_roulis);
	ROS_INFO(" temps_haut_tanga = %f",(float)temps_haut_tangage);*/
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
	erreur_vit[lacet] = commande[lacet] - attitude[5];
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
	commande_devant_droit = saturation(commande_devant_droit ,1100,1900);
	commande_devant_gauche = saturation(commande_devant_gauche,1100,1900);
	commande_derierre_droit = saturation(commande_derierre_droit,1100,1900);
	commande_derierre_gauche = saturation(commande_derierre_gauche,1100,1900);

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


void recuperation_altitude(const drone::Altitude_msg::ConstPtr& _msg, float _altitude[2])
{
	_altitude[0] = _msg->altitude_baro;
	_altitude[1] = _msg->altitude_ultrason;
}
void recuperation_capteur(const drone::Capteurs_msg::ConstPtr& _msg, int *recu_init, float _attitude[6])
{
	*recu_init = 1;
	_attitude[0] = _msg->x;
	_attitude[1] = _msg->y;
	_attitude[2] = fabs(_msg->z);
	_attitude[3] = _msg->vx;
	_attitude[4] = _msg->vy;
	_attitude[5] = _msg->vz;

	if (_attitude[2]<0)
	{
		sens_rotation_z =1;
	}
	else {
		sens_rotation_z =-1;
	}
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


