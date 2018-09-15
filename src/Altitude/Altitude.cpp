#include "Altitude.hpp"
//#include <wiringPiI2C.h>

#define PIN_ECHO 16
#define PIN_PULSE 20

float distance_ultrason;

struct timespec start_echo, stop_echo, diff_echo;
double echo_time;
int i = 1;
float timePrev = 0, timet = 0, timeStep = 0;
struct timeval tv;
float temps_proc = 0;
struct timespec time_actuel, ancien_temps, temps_attente,
		temps_attente_nanosleep;
float temps_recup_altitude = 0;
float frequence = 0;
float altitude_baro = 0;

float sec_to_nano = 1000000000;
int32_t t_fine;
float t; // C
float p; // hPa
float h;       // %
float altitude;                         // meters
int ultrason_validity = 0;

int fd;
int it = 0;
bme280_raw_data raw;
bme280_calib_data cal;

float get_altitude();
void get_altitude_ultrason(void);
void setup();

void setup() {
	temps_attente.tv_sec = 0;
	temps_attente.tv_nsec = 10;

//Altitude
	fd = wiringPiI2CSetup(BME280_ADDRESS);
	if (fd < 0) {
		printf("Device not found");
	}
	// capeur pression
	readCalibrationData(fd, &cal);
	wiringPiI2CWriteReg8(fd, 0xf2, 0x01);   // humidity oversampling x 1
	wiringPiI2CWriteReg8(fd, 0xf4, 0x25); // pressure and temperature oversampling x 1, mode normal-*/

	// Capteur ultrason
	wiringPiSetupGpio();
	pinMode(PIN_PULSE, OUTPUT);
	pinMode(PIN_ECHO, INPUT);
	wiringPiISR(PIN_ECHO, INT_EDGE_RISING, get_altitude_ultrason); /// Mettre des commentaires

	// TODO calibrer valeur instant 0
}

static void printData() {

	// print the data
	printf("distance_ultrason   =%6.6f \n", distance_ultrason);
	printf("altitude  =%6.6f \n",altitude);
	printf("frequence altitude = %6.6f \n", frequence);
}

void loop() {
	clock_gettime(CLOCK_REALTIME, &time_actuel);
	temps_proc = (time_actuel.tv_sec - ancien_temps.tv_sec)
			+ (time_actuel.tv_nsec - ancien_temps.tv_nsec) / 1000000000.0;

	ancien_temps = time_actuel;
	/////////////////////
	frequence = 1 / temps_proc;

		temps_recup_altitude = 0;
		altitude_baro = get_altitude();
		digitalWrite(PIN_PULSE, HIGH);
		sleep(0.00001);
		digitalWrite(PIN_PULSE, LOW);
		clock_gettime(CLOCK_REALTIME, &start_echo);
		printData();


}

void recuperation_initialisation(const std_msgs::String::ConstPtr& msg,
		int *recu_init) // verification initialisation
		{
	*recu_init = 1;
	ROS_INFO("Init recu par Altitude");
}

float get_altitude() {
	// Recupération altitude
	wiringPiI2CWriteReg8(fd, 0xf4, 0x25); // pressure and temperature oversampling x 1, mode normal
	getRawData(fd, &raw);
	t_fine = getTemperatureCalibration(&cal, raw.temperature);
	p = compensatePressure(raw.pressure, &cal, t_fine) / 100; // hPa
	altitude = getAltitude(p);                        // meters
	return altitude;

}

void get_altitude_ultrason(void) {
	ultrason_validity = 1;
	clock_gettime(CLOCK_REALTIME, &start_echo);
	while (digitalRead(PIN_ECHO) && ultrason_validity)
	{
		clock_gettime(CLOCK_REALTIME, &stop_echo);
		if ((stop_echo.tv_nsec - start_echo.tv_nsec) < 0 /* ns */) {
			diff_echo.tv_sec = stop_echo.tv_sec - start_echo.tv_sec - 1;
			diff_echo.tv_nsec = sec_to_nano /* ns */+ stop_echo.tv_nsec
					- start_echo.tv_nsec;
		} else {
			diff_echo.tv_sec = stop_echo.tv_sec - start_echo.tv_sec;
			diff_echo.tv_nsec = stop_echo.tv_nsec - start_echo.tv_nsec;
		}
		echo_time = (float) ((diff_echo.tv_sec * sec_to_nano)
				+ (float) (diff_echo.tv_nsec)); //nsec
		echo_time /= (float) sec_to_nano; //sec
		if (echo_time>0.05)
		{
			ultrason_validity = 0;
		}
	} // On boucle tant que c'est un

	if (ultrason_validity) {
		//calcul de la distance en cm
		distance_ultrason = (float) (echo_time * 17000); //cm
		//ROS_INFO("distance_ultrason =  %f",distance_ultrason);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "altitude");
	ros::NodeHandle n;

	int recu_init = 0;

	ofstream fichier("/home/pi/drone_ws/src/drone/src/donnees_accelero.txt",
			ios::out | ios::trunc);

	boost::function<void(const std_msgs::String::ConstPtr& msg)> temp =
			boost::bind(recuperation_initialisation, _1, &recu_init);
	ros::Subscriber sub = n.subscribe("initialisation", 1, temp);
	drone::Altitude_msg msg_altitude;
	ros::Publisher _pub_msg_altitude = n.advertise < drone::Altitude_msg
			> ("altitude", 1);

	sleep(1);
	setup();
	while (ros::ok()) {
		usleep(200000); // 0.2 secs
		loop();
		if (recu_init == 1) {
			msg_altitude.altitude_baro = altitude;
			msg_altitude.altitude_ultrason = distance_ultrason;
			_pub_msg_altitude.publish(msg_altitude);
		} else {
		}
		ros::spinOnce();
	}

// %EndTag(SPIN)%

	return 0;
}
// %EndTag(FULLTEXT)%
