#include "Capteurs.hpp"

using namespace std;

void setup() {
	temps_attente.tv_sec = 0;
	temps_attente.tv_nsec = 10;

	// initialize device
	printf("Initializing I2C devices...\n");
	//mpu.initialize();

	// verify connection
	printf("Testing device connections...\n");
	int reset = 0;
	mpu.begin(ACCEL_RANGE_4G, GYRO_RANGE_250DPS); // TODO mettre message erreur si raté
}

static void printData() {

	// print the data
	/* printf("ax  =%6.6f\n", ax);
	 printf("ay  =%6.6f\n", ay);
	 printf("az  =%6.6f\n", az);

	 printf("gx   =%6.6f\n", gx);
	 printf("gy   =%6.6f\n", gy);
	 printf("gz   =%6.6f\n", gz);

	 printf("mx  =%6.6f\n", mx);
	 printf("my  =%6.6f\n", my);
	 printf("mz  =%6.6f\n", mz);*/
	//printf("anglez  =%6.6f\n", anglez);
	printf("frequence en Hz  =%f\n", frequence);
	printf("anglex   =%6.6f\n", anglex);
	printf("angley   =%6.6f\n", angley);
	printf("anglez   =%6.6f\n", anglez);
	/*printf("mx  =%6.6f\n", mx);
	 printf("my  =%6.6f\n", my);
	 printf("mz  =%6.6f\n", mz);*/
}
void loop() {
	clock_gettime(CLOCK_REALTIME, &time_actuel);
	temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec)
										+ (time_actuel.tv_nsec - ancien_temps.tv_nsec) / 1000000000.0;

	/*while (temps_proc < 0.001)
		{
			clock_gettime( CLOCK_REALTIME, &time_actuel);
			temps_proc =(time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;
			nanosleep(&temps_attente,&temps_attente_nanosleep);
		}*/
	ancien_temps = time_actuel;
	frequence = 1 / temps_proc;
	mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		ROS_ERROR("magneto HS");
		while (1) sleep(1); // TODO
	}
	// calibration
	calibrate_value();

	/////////////////////
	// FILTRE ROLL PITCH
	filter.begin(frequence,betaRollPitch);
	// FILTRE YAW
	filter_magneto.begin(frequence,betaYaw);

	// update the filter, which computes orientation
	filter.updateIMU(gx, gy, gz, ax, ay, az);

	// update the filter, which computes orientation
	filter_magneto.update(gx, gy, gz, ax, ay, az,mx,my,mz);

	// print the heading, pitch and roll
	anglex = filter.getRoll();
	angley = filter.getPitch();
	anglez = fabs(filter_magneto.getYaw())-180;

	/////////////////////////////

	if (temps_proc != 0) {
		vitx = (anglex - anglex_prec) / temps_proc;
		vity = (angley - angley_prec) / temps_proc;
	}

	anglex_prec = anglex;
	angley_prec = angley;

	if (fabs(anglex) > 380 || fabs(angley) > 380) {
		anglex = 0;
		angley = 0;
	}

	if (temps_recup_altitude < 1) { // TODO changer de nom
		temps_recup_altitude += temps_proc;
	} else {
		temps_recup_altitude = 0;
		printData();
	}

}

void recuperation_initialisation(const std_msgs::String::ConstPtr& msg,
		int *recu_init) // verification initialisation
{
	*recu_init = 1;
	ROS_INFO("Init recu par Capteurs");
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "capteur");
	ros::NodeHandle n;

	int recu_init = 0;

	ofstream fichier("/home/pi/drone_ws/src/drone/src/donnees_accelero.txt",
			ios::out | ios::trunc);

	boost::function<void(const std_msgs::String::ConstPtr& msg)> temp =
			boost::bind(recuperation_initialisation, _1, &recu_init);
	ros::Subscriber sub = n.subscribe("initialisation", 1, temp);
	drone::Capteurs_msg msg_attitude;
	ros::Publisher _pub_msg_attitude = n.advertise < drone::Capteurs_msg
			> ("capteurs", 1);

	sleep(1);
	setup();
	while (ros::ok()) {

		loop();
		if (temps_publication_message<0.002)
		{
			temps_publication_message += temps_proc;
		}
		else
		{
			temps_publication_message=0;
			if (recu_init == 1)
			{
				fichier << " " << ax << " " << ay << " " << az << " " << gy << " "
						<< gx << " " << temps_proc << endl;

				//MDP
				msg_attitude.x = anglex; // tangage
				msg_attitude.y = angley; // roulis
				msg_attitude.z = anglez; // lacet calculer avec le magnetometre
				msg_attitude.vx = -gy;
				msg_attitude.vy = -gx;
				msg_attitude.vz = gz;
				_pub_msg_attitude.publish(msg_attitude);
			}
		}
		ros::spinOnce();
	}

	// %EndTag(SPIN)%

	return 0;
}

void calibrate_value()
{
	gx -= 1.80;
	gy += 1.55;
	gz -= 2.5;
	ax = ax * 0.998149 - 0.6887797;
	ay = ay * 0.99829 - 1.4973;
	az = az * 0.985832 - 1.33133;
	mx += 5.5;
	my -=18;
	mz -= 28;
}
