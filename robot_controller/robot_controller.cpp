/*
   GM RWD season 2
 */
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "std_msgs/Int16.h"

#include <sstream>
#include <pthread.h>

#include <stdio.h>
#include <libevdev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include <libevdev.h>
#include <queue>
#include "std_msgs/String.h"
#include <sstream>
#include "rwd/Speed.h"

#define LEFT_STICK_X 0
#define LEFT_STICK_Y 1
#define RIGHT_STICK_X 2

pthread_t stick_events_tid, comms_tid;
pthread_mutex_t time_lock = PTHREAD_MUTEX_INITIALIZER;

volatile int lx, ly, rz;


#define SCALE_FACTOR 2.5
typedef struct motor_speeds
{
	rwd::Speed lf_speed;
	rwd::Speed lr_speed;
	rwd::Speed rf_speed;
	rwd::Speed rr_speed;
}m_speeds;

std::queue<motor_speeds> speed_queue;
static m_speeds prev_speed;

int FTCMecanumWheel(volatile int lx, volatile int ly, volatile int rz)
{
	m_speeds a_speed;
	if (lx == 0 && ly == 0 && rz == 0)
	{
		return -1;
	}

	double r = sqrt(pow(lx,2) + pow(ly,2));
	double robotAngle = (atan2(ly,lx) - (M_PI / 4));

	//fprintf(out, "%f, %"PRIu64", %d, %d, %d,%f,", secs, nsecs, lx, ly, rz, robotAngle);
	//   printf("%d, %d, %d",  lx, ly, rz);
	double LF = (r * cos(robotAngle) + (double)rz);
	double RF = (r * sin(robotAngle) - (double)rz);
	double LR = (r * sin(robotAngle) + (double)rz);
	double RR = (r * cos(robotAngle) - (double)rz);

	a_speed.lf_speed.speed = (int)LF * SCALE_FACTOR;
	a_speed.rf_speed.speed = (int)RF * SCALE_FACTOR;
	a_speed.lr_speed.speed = (int)LR * SCALE_FACTOR;
	a_speed.rr_speed.speed = (int)RR * SCALE_FACTOR;

	speed_queue.push(a_speed);
	return 0;
}

void aggregate_cb(const ros::TimerEvent&)
{
   //ROS_INFO("aggregate_cb triggered");
   pthread_mutex_lock(&time_lock);
   FTCMecanumWheel(lx, ly, rz);
   lx = ly = rz = 0;
   pthread_mutex_unlock(&time_lock);
}

static void event_handler(struct input_event *ev)
{
	if (ev->type == EV_ABS) {
		//left throttle x
		if (ev->code == LEFT_STICK_X)
		{
			lx = ev->value - 128;
		}

		//right throttle x
		if (ev->code == RIGHT_STICK_X)
		{
			rz = ev->value -128;
		}

		//left throttle y
		if (ev->code == LEFT_STICK_Y)
		{
			ly = ev->value - 128;
		}
	}
}

void* stick_events_thread(void *ptr)
{
    struct libevdev *dev = NULL;
	int fd;
	int rc = 1;
	fd = open("/dev/input/event18", O_RDONLY|O_NONBLOCK);
	if (fd < 0) {
	 fprintf(stderr, "Failed to open device");
	 exit(1);
	}
	rc = libevdev_new_from_fd(fd, &dev);
	if (rc < 0) {
	 fprintf(stderr, "Failed to init libevdev (%s)\n", strerror(-rc));
	 exit(1);
	}
	printf("Input device name: \"%s\"\n", libevdev_get_name(dev));
	printf("Input device ID: bus %#x vendor %#x product %#x\n",
	libevdev_get_id_bustype(dev),
	libevdev_get_id_vendor(dev),
	libevdev_get_id_product(dev));

	do {
		struct input_event ev;
		rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev);
		if (rc == 0)
			event_handler(&ev);
	} while (rc == 1 || rc == 0 || rc == -EAGAIN);
	return NULL;
}

void* comms_thread(void *ptr)
{
	int argc = 0;
	char** argv = NULL;


	lx = ly = rz = 0;

	ros::init(argc, argv, "robot_controller", ros::init_options::NoSigintHandler);
	ros::Publisher cmd_pub;

	ros::NodeHandle n;
    /*std::stringstream ss;
    std_msgs::String cmd;
    cmd.data = ss.str();
	cmd_pub = n.advertise<std_msgs::String>("Cmd", 10);*/
	ros::Publisher lf_pub = n.advertise<rwd::Speed>("LFSpeed", 10);
	ros::Publisher lr_pub = n.advertise<rwd::Speed>("LRSpeed", 10);
	ros::Publisher rf_pub = n.advertise<rwd::Speed>("RFSpeed", 10);
	ros::Publisher rr_pub = n.advertise<rwd::Speed>("RRSpeed", 10);
	ros::Rate loop_rate(25);
	ros::Timer timer1 = n.createTimer(ros::Duration(0.05), aggregate_cb);
	ros::Timer stop_timer = n.createTimer(ros::Duration(3), stop_timer_cb);
	while(ros::ok())
	{
		//cmd_pub.publish(cmd);
		if (!speed_queue.empty())
		{
			pthread_mutex_lock(&time_lock);
			m_speeds a_speed = speed_queue.front();

			a_speed.lf_speed.header.stamp = ros::Time::now();
			lf_pub.publish(a_speed.lf_speed);

			a_speed.lr_speed.header.stamp = ros::Time::now();
			lr_pub.publish(a_speed.lr_speed);

			a_speed.rf_speed.header.stamp = ros::Time::now();
			rf_pub.publish(a_speed.rf_speed);

			a_speed.rr_speed.header.stamp = ros::Time::now();
			rr_pub.publish(a_speed.rr_speed);

			prev_speed = a_speed;
			speed_queue.pop();
			pthread_mutex_unlock(&time_lock);
			printf("LF = %d LR = %d RF = %d RR = %d\n", a_speed.lf_speed.speed
					, a_speed.lr_speed.speed
					, a_speed.rf_speed.speed
					, a_speed.rr_speed.speed);
		}
		else
		{
			prev_speed.lf_speed.header.stamp = ros::Time::now();
			lf_pub.publish(prev_speed.lf_speed);

			prev_speed.lr_speed.header.stamp = ros::Time::now();
			lr_pub.publish(prev_speed.lr_speed);

			prev_speed.rf_speed.header.stamp = ros::Time::now();
			rf_pub.publish(prev_speed.rf_speed);

			prev_speed.rr_speed.header.stamp = ros::Time::now();
			rr_pub.publish(prev_speed.rr_speed);
			printf("---sLF = %d LR = %d RF = %d RR = %d\n", prev_speed.lf_speed.speed
					, prev_speed.lr_speed.speed
					, prev_speed.rf_speed.speed
					, prev_speed.rr_speed.speed);
		}

		ros::spinOnce();
		//Added a delay so not to spam
		loop_rate.sleep();
	}
	return NULL;
}

void* quarterCircleShift(radius, angle)
{
	m_speed a_speed;
	int speed 120; //480 max

	double ratio = speed*radius*cos(angle);

	double LF = (speed * cos(angle) + ratio);
	double RF = (speed * sin(angle) - ratio);
	double LR = (speed * sin(angle) + ratio);
	double RR = (speed * cos(angle) - ratio);

	a_speed.lf_speed.speed = (int)LF * SCALE_FACTOR;
	a_speed.rf_speed.speed = (int)RF * SCALE_FACTOR;
	a_speed.lr_speed.speed = (int)LR * SCALE_FACTOR;
	a_speed.rr_speed.speed = (int)RR * SCALE_FACTOR;
}

/**
 * This sends raw motor values to slave pi over the ROS system.
 */
int main(int argc, char **argv)
{
  pthread_create(&stick_events_tid, NULL, &stick_events_thread, NULL);
  pthread_create(&comms_tid, NULL, &comms_thread, NULL);

  pthread_join(stick_events_tid, NULL);
  pthread_join(comms_tid, NULL);
  return 0;
}

