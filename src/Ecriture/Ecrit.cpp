#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include "paramoteur/Commande.h"
#include "paramoteur/Etat.h"
#include "paramoteur/Position.h"
#include "paramoteur/Psiref.h"
using namespace std;

ofstream fichier_etat("/home/pi/catkin_ws/src/paramoteur/etat_systeme.txt", ios::out | ios::trunc);
void ecriture_etat(const paramoteur::Etat::ConstPtr& msg)
{
 fichier_etat<<  msg->b <<" " << msg->p<<" " <<  msg->r<<" " << msg->phi<<" " <<   msg->psi<<endl;
}
ofstream fichier_psiref("/home/pi/catkin_ws/src/paramoteur/psi_t_ref.txt", ios::out | ios::trunc);
void ecriture_psiref(const paramoteur::Psiref::ConstPtr& msg)
{   
 fichier_psiref<< msg->psiref <<" " <<msg->tref<<endl;
 }
 
ofstream fichier_commande("/home/pi/catkin_ws/src/paramoteur/commande.txt", ios::out | ios::trunc);
void ecriture_commande(const paramoteur::Commande::ConstPtr& msg)
{
fichier_commande<< msg->commande <<endl;
}

ofstream fichier_position("/home/pi/catkin_ws/src/paramoteur/position.txt", ios::out | ios::trunc);
void ecriture_position(const paramoteur::Position::ConstPtr& msg)
{
fichier_position<< msg->x <<" " << msg->y<<" " << msg->z<<endl;
}


// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Ecrit");
  ros::NodeHandle n;
  

// %Tag(SUBSCRIBER)%
ros::Subscriber sub = n.subscribe("systeme2pilotage", 100, ecriture_etat);
ros::Subscriber sub1 = n.subscribe("systeme2guidage", 100, ecriture_position);
ros::Subscriber sub2 = n.subscribe("pilotage", 100, ecriture_commande);
ros::Subscriber sub3 = n.subscribe("guidage", 100, ecriture_psiref);
// %EndTag(SUBSCRIBER)%

  ros::spin();

// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%

