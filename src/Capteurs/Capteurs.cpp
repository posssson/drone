#include "Capteurs.hpp"

using namespace std;
//using namespace Madgwick;



void setup() {
	temps_attente.tv_sec = 0;
	temps_attente.tv_nsec = 10;

	lpFilterX->setBiquad(bq_type_lowpass, 0.00001, 0.707, 0);
	lpFilterY->setBiquad(bq_type_lowpass, 0.00001, 0.707, 0);
	lpFilterZ->setBiquad(bq_type_lowpass, 0.00001, 0.707, 0);

// initialize device
	printf("Initializing I2C devices...\n");
//mpu.initialize();

// verify connection
	printf("Testing device connections...\n");
	int reset = 0;
	mpu.begin(ACCEL_RANGE_8G, GYRO_RANGE_1000DPS); // TODO mettre message erreur si raté
}

static void printData() {

	// print the data
	/*  printf("ax  =%6.6f\n", ax);
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
	printf("anglex   =%6.6f    =%6.6f   =%6.6f\n", anglex, angle_x,roll);
	printf("angley   =%6.6f    =%6.6f   =%6.6f\n", angley, angle_y,pitch);
	printf("anglez   =%6.6f    =%6.6f   =%6.6f\n", anglez, angle_z,heading);
	 printf("mx  =%6.6f\n", mx);
	 printf("my  =%6.6f\n", my);
	 printf("mz  =%6.6f\n", mz);
}
void loop() {
	clock_gettime(CLOCK_REALTIME, &time_actuel);
	temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec)
			+ (time_actuel.tv_nsec - ancien_temps.tv_nsec) / 1000000000.0;

	/* while (temps_proc < 0.001)
	 {
	 clock_gettime( CLOCK_REALTIME, &time_actuel);
	 temps_proc =(time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;
	 nanosleep(&temps_attente,&temps_attente_nanosleep);
	 }*/
	ancien_temps = time_actuel;
	frequence = 1 / temps_proc;
	mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
// calibration
// TODO mettre dans .h
	gx -= 1.80;
	gy += 1.55;
	gz -= 2.5;
	ax = ax * 0.998149 - 0.6887797;
	ay = ay * 0.99829 - 1.4973;
	az = az * 0.985832 - 1.33133;
	mx += 6.27;
	my -=14.0;
	mz -= 35.25;
	/////////////////////
	MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, frequence);

	// TEST MADWICK CPP
	filter.begin(frequence);
    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
	/////////////////////////////

// Application filtre
	ax = LPFAcc_x.apply(ax);
	ay = LPFAcc_y.apply(ay);
	az = LPFAcc_z.apply(az);
	gx = LPFGyro_x.apply(gx);
	gy = LPFGyro_y.apply(gy);

// Calcul angle euler avec filtre compensatoire (fonctionne)
	angle_accel_x = (atan((float) -ax / sqrt((float) az * (float) az))) * 180.0
			/ M_PI;
	angle_accel_y = (atan((float) ay / sqrt((float) az * (float) az))) * 180.0
			/ M_PI;

	anglex = 0.98 * (anglex - (float) (gy) * temps_proc) + 0.02 * angle_accel_x;
	angley = 0.98 * (angley - (float) (gx) * temps_proc) + 0.02 * angle_accel_y;

	///////////////////////////
// Calcul du lacet avec le magnÃ©tometre
	float xh = my*cos(angley/57.295787)+mz*sin(angley/57.295787);
	float yh = mx*cos(anglex/57.295787)+my*sin(angley/57.295787)-mz*sin(anglex/57.295787)*cos(anglex/57.295787);
	anglez =atan2(xh,yh)* 57.29578f;


	angle_x = asin(-2.0f * (q1 * q3 - q0 * q2)) * 57.29578f;
	angle_y = atan2(2*(q0 * q1 + q2 * q3),abs(1.0f - 2*(q1 * q1 + q2 * q2))) * 57.295787;
	angle_z = atan2(2*(q0 * q3 + q1 * q2), 1-2*(q2 * q2 + q3 * q3)) * 57.29578f;

	if (temps_proc != 0) {
		vitx = (anglex - anglex_prec) / temps_proc;
		vity = (angley - angley_prec) / temps_proc;
	}

	anglex_prec = anglex;
	angley_prec = angley;

	if (abs(anglex) > 380 || abs(angley) > 380) {
		anglex = 0;
		angley = 0;
	}

	if (temps_recup_altitude < 1) {
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
		if (recu_init == 1) {
			fichier << " " << ax << " " << ay << " " << az << " " << gy << " "
					<< gx << " " << temps_proc << endl;

//MDP
			msg_attitude.x = anglex; // tangage
			msg_attitude.y = angley; // roulis
			msg_attitude.z = anglez; // lacet calculer avec le magnetometre
			msg_attitude.vx = -gy;
			msg_attitude.vy = -gx;
			_pub_msg_attitude.publish(msg_attitude);
		} else {
		}
		ros::spinOnce();
	}

// %EndTag(SPIN)%

	return 0;
}
// %EndTag(FULLTEXT)%
