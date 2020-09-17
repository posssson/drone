#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include "drone/Data_msg.h"
#include <stdlib.h>
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h> 
#define PORT 22132
using namespace std;
enum {
 tangage,
 roulis,
 lacet,
 altitude,
};

enum {
 IDbarometre,
 IDultrason,
 IDreel,
};
// Ouverture d'un fichier pour stocker les données
ofstream PID("/home/pi/drone_ws/src/drone/src/Data/PID.txt", ios::out | ios::trunc); 
ofstream PID_vit("/home/pi/drone_ws/src/drone/src/Data/PID_Vit.txt", ios::out | ios::trunc); 
ofstream Manette("/home/pi/drone_ws/src/drone/src/Data/Manette.txt", ios::out | ios::trunc); 
ofstream Commande_moteur("/home/pi/drone_ws/src/drone/src/Data/Commande_moteur.txt", ios::out | ios::trunc); 
ofstream Altitude("/home/pi/drone_ws/src/drone/src/Data/Altitude.txt", ios::out | ios::trunc); 
ofstream Data("/home/pi/drone_ws/src/drone/src/Data/Data.txt", ios::out | ios::trunc); 
ofstream Angles("/home/pi/drone_ws/src/drone/src/Data/Angles.txt", ios::out | ios::trunc); 
ofstream PID_tangage("/home/pi/drone_ws/src/drone/src/Data/PID_decompose_tangage.txt", ios::out | ios::trunc); 
ofstream PID_roulis("/home/pi/drone_ws/src/drone/src/Data/PID_decompose_roulis.txt", ios::out | ios::trunc); 
ofstream PID_lacet("/home/pi/drone_ws/src/drone/src/Data/PID_decompose_lacet.txt", ios::out | ios::trunc); 
ofstream PID_altitude("/home/pi/drone_ws/src/drone/src/Data/PID_decompose_altitude.txt", ios::out | ios::trunc); 
ofstream PID_vit_tangage("/home/pi/drone_ws/src/drone/src/Data/PID_vit_decompose_tangage.txt", ios::out | ios::trunc); 
ofstream PID_vit_roulis("/home/pi/drone_ws/src/drone/src/Data/PID_vit_decompose_roulis.txt", ios::out | ios::trunc); 
ofstream PID_vit_lacet("/home/pi/drone_ws/src/drone/src/Data/PID_vit_decompose_lacet.txt", ios::out | ios::trunc); 
ofstream PID_vit_altitude("/home/pi/drone_ws/src/drone/src/Data/PID_vit_decompose_altitude.txt", ios::out | ios::trunc); 
ofstream AccelMag("/home/pi/drone_ws/src/drone/src/Data/AccelMag.txt", ios::out | ios::trunc); 
ofstream Matlab("/home/pi/drone_ws/src/drone/src/Data/Matlab.txt", ios::out | ios::trunc); 
struct timespec startTimeSec;
struct timespec endTimeSec;
int sock = 0; 
int countloop = 0;
float x[] = {0};
// %EndTag(CALLBACK)%
void recuperation_data(const drone::Data_msg::ConstPtr& _msg);
int main(int argc, char **argv)
{
 ros::init(argc, argv, "Ecrit");
 ros::NodeHandle n;

 boost::function<void(const drone::Data_msg::ConstPtr& msg)> temp = boost::bind(recuperation_data, _1);
 ros::Subscriber sub = n.subscribe("data", 10000, temp);

 /*
 struct sockaddr_in serv_addr; 

 if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
 { 
  printf("\n Socket creation error \n"); 
  //return -1; 
 } 
 bzero(&serv_addr,sizeof serv_addr);
 serv_addr.sin_family = AF_INET; 
 serv_addr.sin_port = htons(PORT); 
 // Convert IPv4 and IPv6 addresses from text to binary form 
 if(inet_pton(AF_INET, "192.168.0.152", &serv_addr.sin_addr)<= 0) 
 { 
  printf("\nInvalid address/ Address not supported \n"); 
  //return -1; 
 } 
 printf("\nConnection TCP lancée \n"); 
 while (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
 { 
  printf("\nConnection Failed \n"); 
  sleep(1);
  //return -1; 
 } 
 printf("\nConnection TCP OK !! \n"); 

 while(1)
 {
  
  send(sock, x, sizeof(x), 0);

  printf("Message sent\n"); 
  
  
  sleep(1);
}*/
 //close(sock);

 
 
 ros::spin();
 return 0;
}
// %EndTag(FULLTEXT)%
void recuperation_data(const drone::Data_msg::ConstPtr& _msg)
{
 countloop++;
 //clock_gettime(CLOCK_MONOTONIC, &startTimeSec);
 //Data << _msg->temps_proc <<" " <<(int)(1/_msg->temps_proc) << " "<< (int)_msg->frequence_capteur<< endl;
 //clock_gettime(CLOCK_MONOTONIC, &endTimeSec);
 //printf("temps ecriture = %f \n",((endTimeSec.tv_sec - startTimeSec.tv_sec) + (endTimeSec.tv_nsec - startTimeSec.tv_nsec)/1000000000.0));
 
 clock_gettime(CLOCK_MONOTONIC, &startTimeSec);
 
 PID << _msg->Kp[1] << " " << _msg->Ki[1] << " " << _msg->Kd[1] << " "<< _msg->Kp[2] << " " << _msg->Ki[2] << " " << _msg->Kd[2]<< " "<< _msg->Kp[3] << " " << _msg->Ki[3] << " " << _msg->Kd[3]<<endl;
 PID_vit << _msg->Kp_vit[1] << " " << _msg->Ki_vit[1] << " " << _msg->Kd_vit[1] <<" "<<_msg->Kp_vit[2] << " " << _msg->Ki_vit[2] << " " << _msg->Kd_vit[2] <<" "<<_msg->Kp_vit[3] << " " << _msg->Ki_vit[3] << " " << _msg->Kd_vit[3] <<" "<<endl;
 Manette << _msg->gaz << " " << _msg->consigne[0] << " "<< _msg->consigne[1] << " "<< _msg->consigne[2] << " "<< _msg->consigne[3] <<endl;
 Commande_moteur << _msg->commande_devant_droit-_msg->gaz << " " << _msg->commande_devant_gauche-_msg->gaz << " "<< _msg->commande_derierre_droit-_msg->gaz << " "<< _msg->commande_derierre_gauche-_msg->gaz << endl;
 Altitude << _msg->altitude_capteur[IDbarometre] << " " << _msg->altitude_capteur[IDultrason] << " " << _msg->altitude_capteur[IDreel] << " " << _msg->altitude_vit[0]<< " " << _msg->altitude_vit[1]<<endl;
 Data << _msg->temps_proc <<" " <<(1/_msg->temps_proc) << " "<< _msg->frequence_capteur<< endl;
 
 Angles << _msg->attitude[tangage]<< " " <<_msg->attitude[roulis]<< " " << _msg->attitude[lacet]-_msg->init_lacet<< " " << _msg->attitude[3]<< " " <<_msg->attitude[4]<< " " <<_msg->attitude[5] <<endl;
 
 AccelMag << _msg->accelMag[0]<< " " <<_msg->accelMag[1]<< " " << _msg->accelMag[2]<< " " << _msg->accelMag[3]<< " " <<_msg->accelMag[4]<< " " <<_msg->accelMag[5] <<endl;
 
 PID_tangage << _msg->Kp[tangage] * _msg->erreur[tangage]<< " " <<_msg->Ki[tangage] * _msg->somme_erreurs[tangage] << " " << _msg->Kd[tangage] * _msg->variation_erreur[tangage] <<endl;
 PID_roulis << _msg->Kp[roulis] * _msg->erreur[roulis]<< " " <<_msg->Ki[roulis] * _msg->somme_erreurs[roulis] << " " << _msg->Kd[roulis] * _msg->variation_erreur[roulis] <<endl;
 PID_lacet << _msg->Kp[lacet] * _msg->erreur[lacet]<< " " <<_msg->Ki[lacet] * _msg->somme_erreurs[lacet] << " " << _msg->Kd[lacet] * _msg->variation_erreur[lacet] <<endl;
 PID_altitude << _msg->Kp[altitude] * _msg->erreur[altitude]<< " " <<_msg->Ki[altitude] * _msg->somme_erreurs[altitude] << " " << _msg->Kd[altitude] * _msg->variation_erreur[altitude] <<endl;
 
 PID_vit_tangage << _msg->Kp_vit[tangage] * _msg->erreur_vit[tangage]<< " " <<_msg->Ki_vit[tangage] * _msg->somme_erreurs_vit[tangage] << " " << _msg->Kd_vit[tangage] * _msg->variation_erreur_vit[tangage] <<endl;
 PID_vit_roulis << _msg->Kp_vit[roulis] * _msg->erreur_vit[roulis]<< " " <<_msg->Ki_vit[roulis] * _msg->somme_erreurs_vit[roulis] << " " << _msg->Kd_vit[roulis] * _msg->variation_erreur_vit[roulis] <<endl;
 PID_vit_lacet << _msg->Kp_vit[lacet] * _msg->erreur_vit[lacet]<< " " <<_msg->Ki_vit[lacet] * _msg->somme_erreurs_vit[lacet] << " " << _msg->Kd_vit[lacet] * _msg->variation_erreur_vit[lacet] <<endl;
 PID_vit_altitude << _msg->Kp_vit[altitude] * _msg->erreur_vit[altitude]<< " " <<_msg->Ki_vit[altitude] * _msg->somme_erreurs_vit[altitude] << " " << _msg->Kd_vit[altitude] * _msg->variation_erreur_vit[altitude] <<endl;
 if (countloop == 80)
 {
  Matlab <<_msg->attitude[roulis]<<endl;
  x[0] = {_msg->attitude[roulis]};
  //send(sock, x, sizeof(x), 0);
  countloop = 0;
  // ROS_INFO("DATA SEND");
 }
 clock_gettime(CLOCK_MONOTONIC, &endTimeSec);
 
 //printf("temps ecriture = %f \n",1/(((endTimeSec.tv_sec - startTimeSec.tv_sec) + (endTimeSec.tv_nsec - startTimeSec.tv_nsec)/1000000000.0)));
 
}