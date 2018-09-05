#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone/Capteurs_msg.h"
#include <signal.h>
#include <unistd.h>
using namespace std;

#include "MPU6050_6Axis_MotionApps20.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

						// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

						// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
int first = 0;
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t rx, ry, rz;
float angley = 0, anglex = 0;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void my_handler(int s) {
	printf("Caught signal %d\n", s);
	system("rosnode kill -a");

}
void setup() {
	// initialize device
	printf("Initializing I2C devices...\n");
	mpu.initialize();

	// verify connection
	printf("Testing device connections...\n");
	printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

	// load and configure the DMP
	printf("Initializing DMP...\n");
	devStatus = mpu.dmpInitialize();

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		printf("Enabling DMP...\n");
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		//Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		//attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		printf("DMP ready!\n");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		printf("DMP Initialization failed (code %d)\n", devStatus);
	}
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
	// if programming failed, don't try to do anything
	if (!dmpReady) return;
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	if (fifoCount > 1023) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		printf("FIFO overflow!\n");

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (fifoCount >= 42) {
		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

#ifdef OUTPUT_READABLE_QUATERNION
		// display quaternion values in easy matrix form: w x y z
		mpu.dmpGetQuaternion(&q, fifoBuffer);
	//	printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w, q.x, q.y, q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
		// display Euler angles in degrees
		//mpu.dmpGetQuaternion(&q, fifoBuffer);
		//mpu.dmpGetEuler(euler, &q);
		//ROS_INFO("euler %7.2f %7.2f %7.2f    ", euler[0] * 180 / M_PI, euler[1] * 180 / M_PI, euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		//ROS_INFO("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180 / M_PI, ypr[1] * 180 / M_PI, ypr[2] * 180 / M_PI);
#endif

	/*	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		angley = 0.98*(angley + float(gy)*0.01 / 131) + 0.02*atan2((double)ax, (double)az) * 180 / M_PI; // roulis
		anglex = 0.98*(anglex + float(gx)*0.01 / 131) + 0.02*atan2((double)ay, (double)az) * 180 / M_PI; //tangage*/
		//ROS_INFO("test : %f     %f \n", angley, anglex);
	}
}

void mySigintHandler(int sig)
{
	// Do some custom action.
	// For example, publish a stop message to some other nodes.

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

void recuperation_initialisation(const std_msgs::String::ConstPtr& msg, int *recu_init) // vérification initialisation
{
	*recu_init = 1;
	ROS_INFO("Init recu");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "capteur", ros::init_options::NoSigintHandler);
	ros::NodeHandle n;
	signal(SIGINT, mySigintHandler);
		// quitter programme facile
	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);
	// fin quitter programme facile
	int recu_init = 0;
	setup();


	boost::function<void(const std_msgs::String::ConstPtr& msg)> temp = boost::bind(recuperation_initialisation, _1, &recu_init);
	ros::Subscriber sub = n.subscribe("initialisation", 1, temp);
	drone::Capteurs_msg msg_attitude;
	ROS_INFO("Capteurs");

	ros::Publisher _pub_msg_attitude = n.advertise<drone::Capteurs_msg>("capteurs", 1);
	usleep(100000);
	while (ros::ok())
	{

		loop();
		if (recu_init == 1)
		{

			//MDP
			msg_attitude.x = ypr[0] * 180 / M_PI;
			msg_attitude.y = ypr[1] * 180 / M_PI;
			msg_attitude.z = ypr[2] * 180 / M_PI;

			// FILTRE complementaire
			/*msg_attitude.x = euler[0] * 180 / M_PI;
			msg_attitude.y = angley;
			msg_attitude.z = anglex;*/

			_pub_msg_attitude.publish(msg_attitude);
		}
		ros::spinOnce();
		usleep(5000); // pour eviter d'avoir une surcharge du processeur en attendant que tous les calculs soient finis 
	}

	// %EndTag(SPIN)%

	return 0;
}
// %EndTag(FULLTEXT)%

