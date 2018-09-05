
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include "Biquad.h"
 #include <nlopt.h>
#include <math.h>

using namespace std;

 int nbrFiltre=0;
 double meilleur = 9999999999999999;
 
 double myfunc(unsigned n, const double *x, double *grad, void *my_func_data);
 
int main()
{ cout << "Lancement optimisation filtrage !" << endl;


// SET Optim
double x[2] = {1,0.005}; 

double lb[2] = {1,0.001 }; 
double ub[2] = {8 ,0.5 };
nlopt_opt opt;
opt = nlopt_create(NLOPT_GN_CRS2_LM , 2); /* algorithm and dimensionality */
nlopt_set_lower_bounds(opt, lb);
nlopt_set_upper_bounds(opt,ub);
nlopt_set_min_objective(opt, myfunc, NULL);
nlopt_set_xtol_rel(opt,  0.00001);
double minf; /* `*`the` `minimum` `objective` `value,` `upon` `return`*` */

// TODO a changer ppour mettre ou non l'optim
if (true) {

		if (nlopt_optimize(opt, x, &minf) < 0) {
    printf("nlopt failed!\n");
}
else {
    printf("found minimum at f(%g,%g) = %0.10g\n", x[0], x[1], minf);
}
	}
   return 0;
}



double myfunc(unsigned n, const double *x, double *grad, void *my_func_data)
{		
 float ax,ay,az,gx,gy;
 float angle_acc_x=0,angle_acc_y=0,angle_gyro_x=0,angle_gyro_y=0;
 float somme_erreur_x = 0;
 float somme_erreur_y = 0;
 float somme = 0;
 float temps  =0 ;

	Biquad *lpFilterX = new Biquad();	// create a Biquad, lpFilter;
	Biquad *lpFilterY = new Biquad();	// create a Biquad, lpFilter;
	Biquad *lpFilterZ = new Biquad();	// create a Biquad, lpFilter;


	lpFilterX->setBiquad(bq_type_lowpass,x[1], 0.707, 0);
	lpFilterY->setBiquad(bq_type_lowpass,x[1], 0.707, 0);
	lpFilterZ->setBiquad(bq_type_lowpass,x[1], 0.707, 0);


	
        ifstream fichier("donnees_accelero.txt", ios::in);  // on ouvre le fichier en lecture

        if(fichier)  // si l'ouverture a réussi
        {       
		 //cout << "Fichier ouvert !" << endl;
		for (int i=0;i<19000;i++)
		{
			fichier >> ax >> ay >> az >> gx >> gy >> temps; 

			// FILTRAGE DES VALEURS d'accelero
			for (nbrFiltre = 0;nbrFiltre < x[0];nbrFiltre++)
			{
			
			ax = lpFilterX->process(ax);
			ay = lpFilterY->process(ay);
			az = lpFilterZ->process(az);
			}
			angle_gyro_x = (angle_gyro_x - (float)(gx)*temps);
			angle_gyro_y = (angle_gyro_y - (float)(gy)*temps);
			angle_acc_x = (atan( (float) -ax/sqrt(  (float)az*(float)az ))) * 180.0 / M_PI;
			angle_acc_y = (atan( (float) ay/sqrt(  (float)az*(float)az )  )) * 180.0 / M_PI;
			
			somme_erreur_x += abs(angle_gyro_x - angle_acc_x);
			somme_erreur_y += abs(angle_gyro_y - angle_acc_y);
			

			
        }
				/*cout << "angle_gyro_x ="<<angle_gyro_x << endl;
		 cout << "angle_gyro_y ="<<angle_gyro_y << endl;
		 cout << "angle_acc_x ="<<angle_acc_x << endl;*/
			
	
		
		somme = somme_erreur_x + somme_erreur_y;
		if (somme < meilleur)
		{	
		meilleur = somme;
		cout  << "somme_erreur ="<<somme<< "  x   = "<< std::scientific<<x[1] << endl;
        }
		}
		else
		{
			cout <<"erreur fichier"<<endl;
		}
	return (double)somme;
}