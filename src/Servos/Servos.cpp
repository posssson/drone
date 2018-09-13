
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
			consigne[roulis] =((float)temps_haut_roulis- 1500.0)/10;
			consigne[tangage] =-((float)temps_haut_tangage - 1500.0)/10;
			
			// Mise à zéro si test sans manette
			//consigne[tangage] = 0;
			//consigne[roulis] = 0;

			// RAZ permet de mettre le Ki ou pas.
			if (gaz < 1100)
			{
				raz = 0;
			}
			else
			{
				raz = 1;
			}

			// Limite de gaz
			if (gaz > 1900)
				gaz = 1900;

			// Calcul PID Angles
			calcul_pid_angle();
			
			// Saturation de la commande en angle.
			commande[tangage] = saturation(commande[tangage],-35,35);
			commande[roulis] = saturation(commande[roulis],-35,35);
			
			
			// TODO à enlver pour tester l'angle
			//commande[tangage] = 0;
			//commande[roulis] = 0;

			// Calcul PID vitesse
			calcul_pid_vitesse();
			
			// TODO à enlever
			//commande_vit[tangage] = 0;
			//commande_vit[roulis] = 0;
			//commande_vit[lacet] = 0;
			commande[lacet] = 0;

			// Saturation commadne tangage et roulis
			commande_vit[tangage] = saturation(commande_vit[tangage],-250,250);
			commande_vit[roulis] = saturation(commande_vit[roulis],-250,250);
			
			// Boucle pour afficher des valeurs
			it++;
			if (it ==200) {			
				it = 0;
				
			}
			if (it == 0)
			{
			//ROS_INFO("erreur[roulis] =  %f : erreur_vit[roulis] = %f  commande_vit[roulis] = %f  ",erreur[roulis], erreur_vit[roulis],commande_vit[roulis]);
			//ROS_INFO("commande_vit[tangage] = %f",commande_vit[tangage]);
			//ROS_INFO("commande_vit[roulis] = %f",commande_vit[roulis]);
			//ROS_INFO("temps_haut_arret[roulis] = %f",temps_haut_arret);
			//ROS_INFO("moteur_devant_droit = %f",commande_devant_droit);
			//ROS_INFO("Commande VX =  %f : Commande VY = %f",commande_vit[tangage], commande_vit[roulis]);
			//ROS_INFO("commande[roulis] = %f   attitude[3]= %f",commande[roulis] ,attitude[3]);
			
			//ROS_INFO("temps_proc  =  %lf",temps_proc);
			//ROS_INFO("commande[tangage] = %f   attitude[4]= %f",commande[tangage] ,attitude[4]);
			//ROS_INFO("commande roulis %f",commande[roulis]);
			///ROS_INFO("attitude[tangage] %f",attitude[tangage]);
			//ROS_INFO("attitude[roulis] %f",attitude[roulis]);
			//ROS_INFO("commande_vit[roulis] %f",commande_vit[roulis]);
			//ROS_INFO("commande_vit[tangage] %f",commande_vit[tangage]);
			//ROS_INFO("commande_devant_droit %f",commande_devant_droit);
			//ROS_INFO("commande_derierre_gauche %f",commande_derierre_gauche);
			//ROS_INFO("gaz = %f",gaz);
			}
			
			
			// security
			if (arret == 0 && gaz <1100)
			{
				arret = 1;
				ROS_INFO(" SECURITE ENLEVER");
			}

			calcul_valeur_commande_moteur();

			// Sécurité sur les angles pour les tests 
			if ((abs( attitude[tangage]) > 25 || abs( attitude[roulis])>25) && arret == 1)
				{
					// SECURITE DE TEST
					ROS_INFO("MISE EN SECURITE");
					ROS_INFO("attitude[tangage] = %f",attitude[tangage]);
					ROS_INFO("attitude[roulis] = %f",attitude[roulis]);
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
				it_optim ++;
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
					//ROS_INFO("envoie de l'erreur") ;
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



