#include "Capteurs.hpp"

using namespace std;

void setup() {
	temps_attente.tv_sec = 0;
	temps_attente.tv_nsec = 1;
	// initialize device
	//ROS_INFO("Initializing I2C devices...\n");
	//mpu.initialize();
	
	if (wiringPiSetupGpio() < 0) 
	{
		ROS_ERROR("wiringPiSetupGpio");
	}
	pinMode (27,OUTPUT);
	digitalWrite(27,LOW);
	sleep(1);
	digitalWrite(27,HIGH);
	sleep(3);
	// verify connection
	ROS_INFO("Testing device connections...\n");
	int reset = (int)mpu.begin(ACCEL_RANGE_4G, GYRO_RANGE_500DPS);
	while ( reset != 1)
	{
		ROS_ERROR("ERREUR INIT MPU9250 %d\n",reset);
		digitalWrite(27,LOW);
		sleep(1);
		digitalWrite(27,HIGH);
		sleep(3);
		
		reset = (int)mpu.begin(ACCEL_RANGE_4G, GYRO_RANGE_500DPS);
	}
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	mpu.getMag( &mx, &my, &mz);
	ROS_INFO("mx %f\n",mx);
	ROS_INFO("my %f\n",my);
	ROS_INFO("mz %f\n",mz);
	for (int i = 0; i < 1000;i++)
	{
		mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz,&mx, &my, &mz);
		calibrate_value(1);
		calibrate_value(2);
		calibrate_value(3);
		filter.updateIMU(gx, gy, gz, ax, ay, az);
		filter_magneto.update(gx, gy, gz, ax, ay, az,mx,my,mz);

	}
	ROS_INFO("mx %f\n",mx);
	ROS_INFO("my %f\n",my);
	ROS_INFO("mz %f\n",mz);
	//sleep(100);
}

static void printData() 
{
	//ROS_INFO("frequence en Hz Capteurs = %f\n", frequence);
	//ROS_INFO("correctFrequency = %d\n", correctFrequency);
	/*ROS_INFO("anglex = %f\n", anglex);
	ROS_INFO("angley = %f\n", angley);
	ROS_INFO("anglez = %f\n", anglez);*/
	//mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz,&mx, &my, &mz);

	/*ROS_INFO("angle x = %f\n", anglex);
	ROS_INFO("angle y = %f\n", angley);
	ROS_INFO("angle z = %f\n", anglez);
	
	
	magx = mz * sin(anglex*deg2rad) - my * cos(anglex*deg2rad);
	magy = mx * cos(angley*deg2rad) + my * sin(angley*deg2rad) * sin(anglex*deg2rad) + mz * sin(angley*deg2rad) * cos(anglex*deg2rad);
	yaw = atan2f(magx,magy) * rad2deg;
	// ROS_INFO("yaw = %f\n", yaw);*/
	/*ROS_INFO("ax %f\n",ax);
	ROS_INFO("ay %f\n",ay);
	ROS_INFO("az %f\n",az);
	
	ROS_INFO("gx %f\n",gx);
	ROS_INFO("gy %f\n",gy);
	ROS_INFO("gz %f\n",gz);
	
	*/
	//sleep(1);
	/*ROS_INFO("mx %f\n",mx);
	ROS_INFO("gx %f\n",gx);
	ROS_INFO("anglez = %f\n", anglez);*/

}

void loop() {
	
	//mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	//mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	//clock_gettime( CLOCK_MONOTONIC, &ancien_temps_test);
	//mpu.getGyro(&gx, &gy, &gz);	
	
	//ax = 0; ay = 0; az = 9.81; gx = 0; gy = 0; gz = 0; mx = 0; my = 0; mz = 0;
	//mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	//clock_gettime( CLOCK_MONOTONIC, &temps_actuel_test);
	//temps_test = (temps_actuel_test.tv_sec - ancien_temps_test.tv_sec) + (temps_actuel_test.tv_nsec - ancien_temps_test.tv_nsec)/1000000000.0;	
	//ROS_INFO("temps_test = %f\n", temps_test);
	//mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	calibrate_value(1);
	calibrate_value(2);
	
	if ( (ax == offsetAx) || (ay == offsetAy) || (az == offsetAz) || (gx == offsetGx) || (gy == offsetGy) || (gz == offsetGz) )
	{
		ROS_INFO("Accel = %d   %f   %f   %f",compteur1,ax,ay,az);
		ROS_ERROR("Gyro = %d   %f   %f   %f",compteur2,gx,gy,gz);
	}
	

	// Low Filter

	gx = lowFilterGx.apply(gx);
	gy = lowFilterGy.apply(gy);
	gz = lowFilterGz.apply(gz);
	gx = lowFilterGx2.apply(gx);
	gy = lowFilterGy2.apply(gy);
	gz = lowFilterGz2.apply(gz);
	
	ax = lowFilterAx.apply(ax);
	ay = lowFilterAy.apply(ay);
	az = lowFilterAz.apply(az);
	ax = lowFilterAx2.apply(ax);
	ay = lowFilterAy2.apply(ay);
	az = lowFilterAz2.apply(az);
	
	/*gx = BiquadGx.process(gx);
	gy = BiquadGy.process(gy);
	gz = BiquadGz.process(gz);
	
	ax = BiquadAx.process(ax);
	ay = BiquadAy.process(ay);
	az = BiquadAz.process(az);*/
	
	// Deuxieme filtre smooth
	gx = Last_gx + alpha * ( gx - Last_gx);
	gy = Last_gy + alpha * ( gy - Last_gy);
	gz = Last_gz + alpha * ( gz - Last_gz);
	
	Last_gx = gx;
	Last_gy = gy;
	Last_gz = gz;
	
	ax = Last_ax + alpha * ( ax - Last_ax);
	ay = Last_ay + alpha * ( ay - Last_ay);
	az = Last_az + alpha * ( az - Last_az);
	
	Last_ax = ax;
	Last_ay = ay;
	Last_az = az;
	
	mx = Last_mx + alpha_Mag * ( mx - Last_mx);
	my = Last_my + alpha_Mag * ( my - Last_my);
	mz = Last_mz + alpha_Mag * ( mz - Last_mz);
	
	Last_mx = mx;
	Last_my = my;
	Last_mz = mz;
	/////////////////////
	// FILTRE ROLL PITCH
	filter.begin(frequence,beta);
	// FILTRE YAW
	filter_magneto.begin(frequence,beta);

	// update the filter, which computes orientation
	filter.updateIMU(gx, gy, gz, ax, ay, az);

	// update the filter, which computes orientation
	filter_magneto.update(gx, gy, gz, ax, ay, az,mx,my,mz);

	// print the heading, pitch and roll
	anglex = filter.getRoll();
	angley = filter.getPitch();
	//anglez = sin((fabs(filter_magneto.getYaw())-180)*3.14159265/180)*180; // angle entre -180 et 180 sans discontinuité (ajouter si il pointe à droite ou à gauche) TODO
	//anglez = sin(filter_magneto.getYaw()*0.0174533);
	anglez = filter_magneto.getYaw();
	/////////////////////////////
	
	vitx = (anglex - anglex_prec)/temps_proc;
	vity = (angley - angley_prec)/temps_proc;
	anglex_prec = anglex;
	angley_prec = angley;

	if (fabs(anglex) > 380 || fabs(angley) > 380) {
		anglex = 0;
		angley = 0;
	}

	if (temps_recup_altitude <0.5) { // TODO changer de nom
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
	//ros::Rate rate_capteur(2000); // 8000 hz
	int recu_init = 0;

	_pub_msg_attitude = n.advertise < drone::Capteurs_msg
	> ("capteurs", 1);

	sleep(1);
	boost::function<void(const std_msgs::String::ConstPtr& msg)> temp = 
	boost::bind(recuperation_initialisation, _1, &recu_init);
	ros::Subscriber sub = n.subscribe("initialisation", 1, temp);
	

	Realtime::setup();
	setup();
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		ROS_ERROR("magneto HS");
		ROS_ERROR("ax = %f",ax);
		//sleep(100);
		//setup();
	}
	//sleep(100);
	std::thread worker(ActualisationAccel); 
	std::thread worker2(ActualisationMagn); 
	std::thread worker3(ActualisationCapteurs); 
	//ros::Timer timer1 = n.createTimer(ros::Duration(0.00025), callback1);
	//while (ros::ok()) {
	/*
		loop();
		msg_attitude.x = angley; // tangage
		msg_attitude.y = anglex; // roulis
		msg_attitude.z = anglez; // lacet calculer avec le magnetometre
		msg_attitude.vx = -gx;
		msg_attitude.vy = -gy;
		msg_attitude.vz = -gz;
		msg_attitude.frequence_capteur = frequence;
		capteur << -gy << " " <<vitx << " " << -gx << " "<< vity << " " << angley<< " " << anglex<< " " << temps_proc <<endl;
		_pub_msg_attitude.publish(msg_attitude);
*///clock_gettime( CLOCK_MONOTONIC, &ancien_temps_test);
	//usleep(1);
	
	
	//ros::spinOnce();
	//rate_capteur.sleep();
	//}
	ros::spin();
	
	// %EndTag(SPIN)%

	return 0;
}
void ActualisationCapteurs()
{ 
	Realtime::cpu2();
	Realtime::realTimeSched();
	
	ros::Rate rate(frequenceSet); // ROS Rate at 1000Hz
	while (ros::ok()) {
		clock_gettime(CLOCK_MONOTONIC, &time_actuel);
		temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec)
		+ (time_actuel.tv_nsec - ancien_temps.tv_nsec) / 1000000000.0;
		
		if (temps_proc < time_loop*0.9)
		{
			correctFrequency = (int)((time_loop - temps_proc)*1000000);
			if (correctFrequency>120)
			{
				correctFrequency -= 40;
				usleep(correctFrequency);
			}
			while (temps_proc < time_loop*0.9)
			{
				clock_gettime( CLOCK_MONOTONIC, &time_actuel);
				temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;	
			}
			clock_gettime( CLOCK_MONOTONIC, &time_actuel);
			temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec) + (time_actuel.tv_nsec - ancien_temps.tv_nsec)/1000000000.0;	
		}
		
		
		
		ancien_temps = time_actuel;
		frequence = 1 / temps_proc;
		
		loop();
		msg_attitude.roll = angley; // tangage
		msg_attitude.pitch = anglex; // roulis
		msg_attitude.yaw = anglez; // lacet calculer avec le magnetometre
		msg_attitude.gx = -gx;
		msg_attitude.gy = -gy;
		msg_attitude.gz = -gz;
		msg_attitude.ax = ax;
		msg_attitude.ay = ay;
		msg_attitude.az = az;
		msg_attitude.mx = mx;
		msg_attitude.my = my;
		msg_attitude.mz = mz;
		msg_attitude.frequence_capteur = frequence;
		_pub_msg_attitude.publish(msg_attitude);

		//ros::spinOnce();
		rate.sleep();
	}
}
void ActualisationAccel()
{
	ros::Rate rate(1000); // ROS Rate at 100Hz
	
	while (ros::ok()) {
		/*
		mpu.getAccel(&ax, &ay, &az);
		calibrate_value(2);*/
		rate.sleep();
	}
}

void ActualisationMagn()
{
	ros::Rate rate(100); // ROS Rate at 100Hz
	
	while (ros::ok()) {
		mpu.getMag(&mx, &my, &mz);	
		if ((mx == 0.0f) || (my == 0.0f) || (mz == 0.0f))
		{
			//ROS_ERROR("magneto HS");
			//mpu.resetAK8963();
		}
		else{
			calibrate_value(3);
		}
		rate.sleep();
	}
	
}
void calibrate_value(int i)
{
	
	// Suppression des valeurs 0
	

	
	if (i == 1){
		if (gx == 0.0f) 
		{
			gx = Last_gx - offsetGx;
			//ROS_INFO("gx");
		}
		if (gy == 0.0f)
		{
			gy = Last_gy - offsetGy;
			//ROS_INFO("gy");
		}
		if (gz == 0.0f) 
		{
			gz = Last_gz - offsetGz;
			//ROS_INFO("gz");
		}
		gx += offsetGx;
		gy += offsetGy;
		gz += offsetGz;
	}
	if (i == 2){
		if (ax == 0.0f) 
		{
			ax = Last_ax - offsetAx;
			//ROS_INFO("ax");
		}
		if (ay == 0.0f)
		{
			ay = Last_ay - offsetAy;
			//ROS_INFO("ay");
		}
		if (az == 0.0f)
		{
			az = Last_az - offsetAz;
			//ROS_INFO("az");
		}
		ax += offsetAx;
		ay += offsetAy;
		az += offsetAz;
	}
	if (i == 3){
		if (mx == 0.0f)
		{
			mx = Last_mx - offsetMx;
			//ROS_INFO("mx");
		}
		if (my == 0.0f) 
		{
			my = Last_my - offsetMy;
			//ROS_INFO("my");
		}
		if (mz == 0.0f)
		{
			mz = Last_mz - offsetMz;
			//ROS_INFO("mz");
		}
		mx += offsetMx;
		my += offsetMy;
		mz += offsetMz;
	}
}
