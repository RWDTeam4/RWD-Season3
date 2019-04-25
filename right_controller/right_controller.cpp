/*
   GM RWD season 2
 */

#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <wiringPi.h>
#include "rwd/Speed.h"

#include <boost/thread.hpp>
#include <pthread.h>
#include <math.h>

/**
 * This runs on a pi that controls left side motors, receives the speeds for
 * Left front and Right rear motors over the ROS communication channel.
 * ROS master actually is run on Linux machine that interfaces to bluetooth Joystick controller
 */

ros::Duration d(0.01);

#define FORWARD (HIGH)
#define REVERSE (LOW)
#define MAX_SPEED (480U) //19.2 MHz / 2 / 480 = 20 kHz
#define MIN_SPEED (40U)
#define SCALE_FACTOR 0.45
enum motor_label
{
	MOTOR1,
	MOTOR2
};
const int motor1PWMpin = 26;
const int motor2PWMpin = 23;
const int motor1dirPin = 5;
const int motor1enPin =  3;
const int motor2dirPin = 6;
const int motor2enPin =  4;
pthread_mutex_t time_lock = PTHREAD_MUTEX_INITIALIZER;
volatile bool dont_stop = false;
int prev_speed1 = 0;
int prev_speed2 = 0;

static inline void enableMotor(volatile int motorPin)
{
	digitalWrite(motorPin, HIGH);
}

static inline void disableMotor(volatile int motorPin)
{
	digitalWrite(motorPin, LOW);
}

static inline void setSpeed(motor_label motor_label, volatile int speed)
{
	int dirPin, enPin, pwmPin;
	if (motor_label == MOTOR1)
	{
		dirPin = motor1dirPin;
		enPin = motor1enPin;
		pwmPin = motor1PWMpin;
		pthread_mutex_lock(&time_lock);
		if (prev_speed1 < speed)
		{
			speed = prev_speed1 + (speed - prev_speed1) * 0.3;
		}
		else if (prev_speed1 >= speed)
		{
			speed = prev_speed1 - (prev_speed1 - speed) * 0.3;
		}
		prev_speed1 = speed;

		pthread_mutex_unlock(&time_lock);
		if (speed < 0)
		{
			speed = -speed;
			digitalWrite(dirPin, REVERSE);
			delay(50);
		}
		else
		{
			digitalWrite(dirPin, FORWARD);
			delay(50);
		}
	}
	else if (motor_label == MOTOR2)
	{
		dirPin = motor2dirPin;
		enPin = motor2enPin;
		pwmPin = motor2PWMpin;

		pthread_mutex_lock(&time_lock);
		speed = prev_speed2 + (speed - prev_speed2) * SCALE_FACTOR;
		prev_speed2 = speed;
		pthread_mutex_unlock(&time_lock);

		if (speed < 0)
		{
			speed = -speed;
			digitalWrite(dirPin, FORWARD);
			delay(50);
		}
		else
		{
			digitalWrite(dirPin, REVERSE);
			delay(50);
		}
	}
	enableMotor(enPin);
	if (speed > MAX_SPEED)
	{
		speed = MAX_SPEED;
	}
	/*if (speed != 0 && speed < MIN_SPEED)
	{
		speed += MIN_SPEED;
	}*/

	pwmWrite(pwmPin, speed * SCALE_FACTOR);
	delay(50);
}

void stop_timer_cb(const ros::TimerEvent&)
{
	int speed1;
	int speed2;
	static int cnt = 0;
	if (dont_stop)
	{
		dont_stop = false;
		return;
	}
	speed1 = speed2 = 0;
	speed1 = prev_speed1;
	speed2 = prev_speed2;

	while (abs(speed1) || abs(speed2))
	{
		if (abs(speed1) <= MIN_SPEED || cnt > 4 || abs(speed2) <= MIN_SPEED)
		{
			cnt = 0;
			speed1 = speed2 = 0;
			prev_speed1 = prev_speed2 = 0;
			setSpeed(MOTOR1, speed1);
			setSpeed(MOTOR2, speed2);
			printf("hard break\n");
			break;
		}
		//speed1 = log2(abs(speed1));
		//speed2 = log2(abs(speed2));
		if (prev_speed1 < 0)
		{
			//to slow down from speeds with -ve sign
			speed1 += 20;
		}
		else
		{
			speed1 -= 20;
		}
		if (prev_speed2 < 0)
		{
			//to slow down from speeds with -ve sign
			speed2 += 20;
		}
		else
		{
			speed2 -= 20;
		}
		printf("speed1 = %d, speed2 = %d\n", speed1, speed2);
		setSpeed(MOTOR1, speed1);
		setSpeed(MOTOR2, speed2);
		cnt++;
	}
	prev_speed1 = 0;
	prev_speed2 = 0;
}

inline static void motors_init()
{
	pinMode(motor1PWMpin, PWM_OUTPUT); /* set PWM 12 pin as output */
	pinMode(motor2PWMpin, PWM_OUTPUT); /* set PWM 12 pin as output */
	pwmSetMode(PWM_MODE_MS); /* Set PWM Mode Mark-Space */
	pwmSetClock(2);  /* Set Clock Divider (19.2 MHz / 2) */
	pwmSetRange(MAX_SPEED);	 /* 19.2 MHz / 2 / 480 = 20 kHz */

	pinMode(motor1dirPin, OUTPUT);
	pinMode(motor1enPin, OUTPUT);
	pinMode(motor2dirPin, OUTPUT);
	pinMode(motor2enPin, OUTPUT);

	digitalWrite(motor2dirPin, FORWARD);
	digitalWrite(motor1dirPin, FORWARD);

	disableMotor(motor1enPin);
	disableMotor(motor2enPin);
}

void rightCallback(const rwd::Speed::ConstPtr& motor1, const rwd::Speed::ConstPtr& motor2)
{
	//ROS_INFO_STREAM("rightCallback speed1: [" << motor1->speed << "] [thread=" << boost::this_thread::get_id() << "]");
	//pthread_mutex_lock(&time_lock);
	dont_stop = true;
	//pthread_mutex_unlock(&time_lock);

   //ROS_INFO_STREAM("rightCallback speed2: [" << motor2->speed << "] [thread=" << boost::this_thread::get_id() << "]");

  //ROS_INFO_STREAM("leftCallbackTime: [" << motor1->header.stamp.toSec() << "] [thread=" << boost::this_thread::get_id() << "]");
  //ROS_INFO_STREAM("leftCallbackTime: [" << motor2->header.stamp.toSec() << "] [thread=" << boost::this_thread::get_id() << "]");
  setSpeed(MOTOR1, motor1->speed);
  setSpeed(MOTOR2, motor2->speed);
  d.sleep();
}

void cmdCallback()
{
	ROS_INFO_STREAM("cmdCallback time [" << ros::WallTime::now() << "]");
}

/*
void forwardHandler()
{
	for
}*/

int main(int argc, char **argv)
{
  int ret = wiringPiSetup();
  if (ret == -1)
  {
	  printf("WiringPi Init Failed ret = %d\n", ret);
      exit (1) ;
  }
  motors_init();

  ros::init(argc, argv, "rightController");
  ros::NodeHandle n;
  ros::Timer stop_timer = n.createTimer(ros::Duration(0.3), stop_timer_cb);

  message_filters::Subscriber<rwd::Speed> lf_sub(n, "RFSpeed", 10);
  message_filters::Subscriber<rwd::Speed> lr_sub(n, "RRSpeed", 10);

  typedef message_filters::sync_policies::ApproximateTime<rwd::Speed, rwd::Speed> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), lf_sub, lr_sub);
  sync.registerCallback(boost::bind(rightCallback, _1, _2));

  /**
   * The AsyncSpinner object allows you to specify a number of threads to use
   * to call callbacks.  If no explicit # is specified, it will use the # of hardware
   * threads available on your system.  Here we explicitly specify 4 threads.
   */
  ros::AsyncSpinner s(1);
  s.start();

  ros::Rate r(30);
  while (ros::ok())
  {
    r.sleep();
  }

  return 0;
}

