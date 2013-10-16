#include "ros/ros.h"
#include "motors/polarcoord.h"
#include "motors/wheel_speed.h"
#include <cmath>

ros::Subscriber camera_sub;
ros::Publisher speed_pub;

void camera_cb(const motors::polarcoord &hand){
	printf("angle: %f,distance: %f\n",hand.angle,hand.distance);

	motors::wheel_speed desiredSpeed;
	desiredSpeed.W1 = 2.0*hand.distance*(1+sin(0.5*hand.angle));
	desiredSpeed.W2 = 2.0*hand.distance*(1-sin(0.5*hand.angle));

	speed_pub.publish(desiredSpeed);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "AngleToXY"); //Creates a node named "KeyboardEvent"
	ros::NodeHandle n;

	camera_sub = n.subscribe("/hand/location", 1, camera_cb);
	speed_pub = n.advertise<motors::wheel_speed>("/desired_speed", 1);


	ros::spin();
}
