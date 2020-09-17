#include "Altitude.hpp"
//#include <wiringPiI2C.h>

float altitude[2],vitesse_altitude[2],altitude_capteur[2],altitude_precedent[2],altitude_init[2];       // meters
int ultrason_validity = 0;
enum {
	IDbarometre,
	IDultrason,
};
int fd;
int it = 0;
bme280_raw_data raw;

float get_altitude();
void setup();

void setup() {
	/* if (wiringPiSetupGpio() = -1)
	{
		while(1)
			ROS_INFO("ERREUR ALTITUDE INIT\n");
		
	}*/
	temps_attente.tv_sec = 0;
	temps_attente.tv_nsec = 10;
	
	//Altitude
	if (bme.begin() < 0)
	{
		ROS_ERROR("Init BME failed !! ");
	}
	sonar.init(PIN_PULSE, PIN_ECHO);
}

static void printData() {
	if (false)
	{
		ROS_INFO("altitude[IDbarometre] = %6.6f \n",altitude[IDbarometre]);
		ROS_INFO("altitude[IDultrason]  = %6.6f \n", altitude[IDultrason]);
		ROS_INFO("frequence altitude = %6.6f \n", frequence);
		// print the data
		//
		//ROS_INFO("raw.pressure  = %6.6f \n", value.pressure);
		/*ROS_INFO("altitude = %6.6f \n",altitude[IDbarometre]);
		ROS_INFO("altitude_capteur = %6.6f \n",altitude_capteur[IDbarometre]);
		ROS_INFO("init = %6.6f \n",altitude_init[IDbarometre]);
		ROS_INFO("gaz = %6.6f \n",gaz);
		ROS_INFO("value.pressure = %6.6f \n",value.pressure);*/
		
		
		
		//ROS_INFO("vitesse altitude = %6.6f \n",vitesse_altitude[IDbarometre]);
		//
		//ROS_INFO("altitude_capteur = %6.6f \n",altitude_capteur[IDbarometre]);
	}
}

void loop() {
	clock_gettime(CLOCK_MONOTONIC, &time_actuel);
	temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec)
	+ (time_actuel.tv_nsec - ancien_temps.tv_nsec) / 1000000000.0;

	ancien_temps = time_actuel;
	/////////////////////
	frequence = 1 / temps_proc;

	temps_recup_altitude = 0;
	altitude_capteur[IDbarometre] = get_altitude();
	if (gaz < 1200)
	{
		altitude_init[IDbarometre] = altitude_capteur[IDbarometre];
		altitude_init[IDultrason] = altitude_capteur[IDultrason];
	}
	
	altitude[IDbarometre] = altitude_capteur[IDbarometre]-altitude_init[IDbarometre];
	altitude[IDultrason] = altitude_capteur[IDultrason]-altitude_init[IDultrason];
	
	if ( altitude[IDultrason] < 2 && altitude[IDultrason] > 0.001)
	{
		correction_altitude = altitude[IDultrason] - altitude[IDbarometre];
	}
	
	altitude[IDbarometre] += correction_altitude;
	
	vitesse_altitude[IDbarometre] = (altitude[IDbarometre]-altitude_precedent[IDbarometre])/(float)temps_proc;
	vitesse_altitude[IDultrason] = (altitude[IDultrason]-altitude_precedent[IDultrason])/(float)temps_proc;
	altitude_precedent[IDbarometre] = altitude[IDbarometre];
	altitude_precedent[IDultrason] = altitude[IDultrason];

	printData();


}

void recuperation_initialisation(const std_msgs::String::ConstPtr& msg,
int *recu_init) // verification initialisation
{
	*recu_init = 1;
	ROS_INFO("Init recu par Altitude");
}

float get_altitude() {
	// Recupération altitude 10 fois plus moyenne
	bme.getRawData(&value);
	altitude_capteur[IDbarometre] = bme.getAltitude(value.pressure);

	altitude_capteur[IDultrason] = sonar.distance(0.023);
	return altitude_capteur[IDbarometre];
}

void recuperation_gaz(const drone::Gaz_msg::ConstPtr& _msg, float *_gaz)
{
	*_gaz = _msg->gaz;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "altitude");
	ros::NodeHandle n;
	
	int recu_init = 0;
	boost::function<void(const drone::Gaz_msg::ConstPtr& msg)> temp1 = boost::bind(recuperation_gaz, _1, &gaz);
	ros::Subscriber sub1 = n.subscribe("gaz", 1, temp1);

	boost::function<void(const std_msgs::String::ConstPtr& msg)> temp = 
	boost::bind(recuperation_initialisation, _1, &recu_init);
	ros::Subscriber sub = n.subscribe("initialisation", 1, temp);
	drone::Altitude_msg msg_altitude;
	ros::Publisher _pub_msg_altitude = n.advertise < drone::Altitude_msg> ("altitude", 1);
	ROS_INFO("ALTITUDE Lancemeent setup");
	setup();
	ROS_INFO("ALTITUDE setup ok !");
	while (ros::ok()) {
		usleep(40000);
		loop();

		
		//ROS_INFO("frequence altitude = %6.6f \n", frequence);
		//ROS_INFO("altitude = %6.6f \n",altitude[IDultrason]);
		
		if (recu_init == 1) {
			
			msg_altitude.altitude_baro = altitude[IDbarometre];
			msg_altitude.altitude_ultrason = altitude[IDultrason];
			msg_altitude.vitesse_altitude_baro = vitesse_altitude[IDbarometre];
			msg_altitude.vitesse_altitude_ultrason = vitesse_altitude[IDultrason];
			_pub_msg_altitude.publish(msg_altitude);
		} 
		else {
			
		}
		ros::spinOnce();
	}

	// %EndTag(SPIN)%

	return 0;
}
// %EndTag(FULLTEXT)%
