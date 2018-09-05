#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone/Capteurs_msg.h"
#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <fstream>
using namespace std;

#include "I2Cdev.h"
#include "MPU9250.h"
#include <wiringPi.h>
#include "../Common/LowPassFilter2p.hpp"
#include "../Common/Biquad.h"
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

MPU9250 mpu(0x68);
math::LowPassFilter2p LPFAcc_x(1000,50);
math::LowPassFilter2p LPFAcc_y(1000,50);
math::LowPassFilter2p LPFAcc_z(1000,50);
math::LowPassFilter2p LPFGyro_x(1000,50);
math::LowPassFilter2p LPFGyro_y(1000,50);
float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;
float t; // temperature
int i = 1;
float gyroScale = 131;
float arx, ary, arz, grx, gry, grz, gsx, gsy, gsz;
float timePrev = 0, timet = 0, timeStep = 0;
float ypr[3];
struct timeval  tv;
float anglex = 0, angley = 0, anglez = 0;
float temps_proc =0;
float anglex_prec  = 0, angley_prec = 0, anglez_prec = 0;
float vitx = 0,vity = 0;
float angle_accel_x = 0,angle_accel_y = 0;
struct timespec time_actuel,ancien_temps,temps_attente,temps_attente_nanosleep;

 Biquad *lpFilterX = new Biquad();	// create a Biquad, lpFilter;
 Biquad *lpFilterY = new Biquad();	// create a Biquad, lpFilter;
 Biquad *lpFilterZ = new Biquad();	// create a Biquad, lpFilter;


int it = 0;
	

 
void setup()
{
	temps_attente.tv_sec = 0;
	temps_attente.tv_nsec = 10;
	
	
	lpFilterX->setBiquad(bq_type_lowpass,0.00001, 0.707, 0);
	lpFilterY->setBiquad(bq_type_lowpass,0.00001, 0.707, 0);
	lpFilterZ->setBiquad(bq_type_lowpass,0.00001, 0.707, 0);
	
	// initialize device
	printf("Initializing I2C devices...\n");
	//mpu.initialize();
	
	// verify connection
	printf("Testing device connections...\n");
	int reset = 0;
	mpu.begin(ACCEL_RANGE_8G,GYRO_RANGE_1000DPS);
	
}

static void printData(){

  // print the data
  printf("%6.6f\t", ax);
  printf("%6.6f\t", ay);
  printf("%6.6f\t", az);

  printf("%6.6f\t", gx);
  printf("%6.6f\t", gy);
  printf("%6.6f\t", gz);

  printf("%6.6f\t", mx);
  printf("%6.6f\t", my);
  printf("%6.6f\t", mz);

}
void loop()
{

		clock_gettime( CLOCK_REALTIME, &time_actuel);
		temps_proc =(time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;
		
		/*while (temps_proc < 0.001)
		{
			clock_gettime( CLOCK_REALTIME, &time_actuel);
			temps_proc =(time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;
			nanosleep(&temps_attente,&temps_attente_nanosleep);
			printf("temps_proc        %f\n", temps_proc);
		}*/
		ancien_temps = time_actuel;

	mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz,&mx,&my,&mz);
		
	// calibration
	// TODO mettre dans .h
	gx -= 1.80;
	gy += 1.55;
	gz -= 0.78;
	ax = ax*0.997505 - 0.73333;
	ay = ay*0.998909 - 1.07523;
	az = az*0.986718 - 0.897756;

	
	ax= LPFAcc_x.apply(ax);
	ay= LPFAcc_y.apply(ay);
	az= LPFAcc_z.apply(az);
	gx= LPFGyro_x.apply(gx);
	gy= LPFGyro_y.apply(gy);
	
	//printf("LPFAcc_x        %f\n", LPFAcc_x.apply(az));
	
	angle_accel_x = (atan( (float) -ax/sqrt(  (float)az*(float)az ))) * 180.0 / M_PI;
	angle_accel_y = (atan( (float) ay/sqrt(  (float)az*(float)az )  )) * 180.0 / M_PI;
	anglex = 0.98*(anglex - (float)(gy)*temps_proc )  + 0.02 * angle_accel_x;
	angley = 0.98*(angley - (float)(gx)*temps_proc )  + 0.02 * angle_accel_y;
	
//printf("anglex        %f\n", anglex);
	
	anglez = (anglez + float(gz)*temps_proc );
if(temps_proc != 0) {
	vitx = (anglex - anglex_prec)/temps_proc;
	vity = (angley - angley_prec)/temps_proc;
}
	anglex_prec = anglex;
	angley_prec = angley;
	
	if (abs(anglex) >380 || abs(angley) >380)
	{
		anglex = 0;
		angley = 0;
	}
	
	/*printf("anglex        %f\n", anglex);
	printf("accelx        %f\n", ax);
	printf("accely        %f\n", ay);
	printf("accelz        %f\n", az);
	printf("ax                %f\n", ax);
	printf("avant_filtrage_ax %f\n", avant_filtrage_ax);*/
	
	/*printf("angle_accel_x %f\n", angle_accel_x);
	printf("anglex        %f\n", anglex);
	printf("angle_accel_y %f\n", angle_accel_y);
	printf("angley        %f\n", angley);*/
	
/*/std::cout <<""<< std::endl;
std::cout << "angle ax =  "<<anglex << std::endl;
std::cout << "angle ay =  "<<angley << std::endl;*/
/*std::cout << "angle gx =  "<<gx << std::endl;
std::cout << "angle gy =  "<<gy << std::endl;
  printf("ax %f\t", ax);
  printf("ay %f\t", ay);
  printf("az %f\t", az);*/
  //printf("angle gx vitx = %f    %f\n",gy,vitx);
  //printf("angle gy vity = %f    %f\n",-gx,vity);
	//ROS_INFO("angle roulis tangage vit roulis tangage  %f  %f %f  %f", anglex, angley, vitx,vity);
}

void recuperation_initialisation(const std_msgs::String::ConstPtr& msg, int *recu_init) // vérification initialisation
{
	*recu_init = 1;
	ROS_INFO("Init recu");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "capteur");
	ros::NodeHandle n;
	
	setup();

	int recu_init = 0;

ofstream fichier("/home/pi/drone_ws/src/drone/src/donnees_accelero.txt", ios::out | ios::trunc); 

	boost::function<void(const std_msgs::String::ConstPtr& msg)> temp = boost::bind(recuperation_initialisation, _1, &recu_init);
	ros::Subscriber sub = n.subscribe("initialisation", 1, temp);
	drone::Capteurs_msg msg_attitude;
	ROS_INFO("Capteurs");

	ros::Publisher _pub_msg_attitude = n.advertise<drone::Capteurs_msg>("capteurs", 1);
	
	while (ros::ok())
	{
		loop();
		if (recu_init == 1)
		{
			fichier << " " << ax << " " << ay << " "<<az << " " << gy << " " << gx<<" " << temps_proc <<endl;

			//MDP
			msg_attitude.x = anglex; // tangage
			msg_attitude.y = angley; // roulis 
			msg_attitude.z = 0;
			msg_attitude.vx = -gy;  
			msg_attitude.vy = -gx ;
			//printData();
			_pub_msg_attitude.publish(msg_attitude);
		}
		ros::spinOnce();
		}

	// %EndTag(SPIN)%

	return 0;
}
// %EndTag(FULLTEXT)%

