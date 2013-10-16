#include "ros/ros.h"
#include <std_msgs/Int32.h>

#include "differential_drive/KeyEvent.h"
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>
#include <differential_drive/Speed.h>

#include <visualization_msgs/Marker.h>
#include "MovementControl.h"
#include "motors/polarcoord.h"

#include <string.h>
#include <math.h>
#include <SDL/SDL.h>

using namespace differential_drive;
ros::Subscriber enc_sub;
ros::Subscriber key_sub;
ros::Subscriber hand_sub;
ros::Publisher pwm_pub;
ros::Publisher marker_pub;
// Movement data
MovementControl movementController;

void publish_marker_data() {
	// Visualization stuff
	visualization_msgs::Marker marker;
	visualization_msgs::Marker marker_target;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/my_frame";
	marker_target.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();
	marker_target.header.stamp = ros::Time::now();
	// Sets namespace and ID
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker_target.ns = "basic_shapes";
	marker_target.id = 1;
	// Sets the type of visual to cube; and add as action. Add also works as update.
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker_target.type = visualization_msgs::Marker::CUBE;
	marker_target.action = visualization_msgs::Marker::ADD;

	// Position
	Coord robot = movementController.getRobotPos();
	marker.pose.position.x = robot.x;
	marker.pose.position.y = robot.y;
	marker.pose.position.z = 0;

	Coord target = movementController.getTargetPos();
	marker_target.pose.position.x = target.x;
	marker_target.pose.position.y = target.y;
	marker_target.pose.position.z = 0;

	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = sin(
			movementController.getRobotDirection() * M_PI / 360);
	marker.pose.orientation.w = cos(
			movementController.getRobotDirection() * M_PI / 360);
	marker_target.pose.orientation.x = 0;
	marker_target.pose.orientation.y = 0;
	marker_target.pose.orientation.z = 0;
	marker_target.pose.orientation.w = 1;
	PWM pwm;
	// Green
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	marker_target.color.r = 1.0f;
	marker_target.color.g = 0.0f;
	marker_target.color.b = 0.0f;
	marker_target.color.a = 1.0;
	// large scale
	marker.scale.x = 10.0;
	marker.scale.y = 10.0;
	marker.scale.z = 10.0;
	marker_target.scale.x = 10.0;
	marker_target.scale.y = 10.0;
	marker_target.scale.z = 10.0;
	// I WANNA LIVE FOREVER
	marker.lifetime = ros::Duration();
	marker_target.lifetime = ros::Duration();
	marker_pub.publish(marker);
	marker_pub.publish(marker_target);
}

void receive_key(const KeyEvent::ConstPtr &msg) //Taken from KeyboardControl.cpp
		{
	switch (msg->sym) {
	case SDLK_UP:
		if (msg->pressed) {
			movementController.setTargetRelative(Coord(100, 0));
		}
		break;
	case SDLK_DOWN:
		if (msg->pressed) {
			movementController.setTargetRelative(Coord(-100, 0));
		}
		break;
	case SDLK_LEFT:
		if (msg->pressed) {
			movementController.setTargetRelative(Coord(0, 100));
		}
		break;
	case SDLK_RIGHT:
		if (msg->pressed) {
			movementController.setTargetRelative(Coord(0, -100));
		}
		break;
	case SDLK_SPACE:
		if (msg->pressed) {
			movementController.setTargetRelative(Coord(0, 0));
		}
		break;
	}
}

void run(float deltaY, float deltaX) {
	printf("Y: %f, X: %f\n", deltaY, deltaX);
	static ros::Time t_start = ros::Time::now();
	//int timestamp = msg->timestamp;
	//printf("%d:got encoder L:%d , false R:%d\n", timestamp, left, right);

	Coord encoderInfo = movementController.moveTowardsTarget(
			Coord(deltaY, deltaX));
	Speed speed;
	speed.W1 = (float) encoderInfo.y;
	speed.W2 = (float) encoderInfo.x;
	/*PWM pwm;
	 pwm.PWM1 = encoderInfo.y;
	 pwm.PWM2 = encoderInfo.x;
	 pwm.header.stamp = ros::Time::now();*/
	pwm_pub.publish(speed);
	publish_marker_data();
}

//Callback function for the "/encoder" topic. Prints the enconder value and changes the speed accordingly.
void receive_encoder(const Encoders::ConstPtr &msg) {
	//run(msg->delta_encoder1, msg->delta_encoder2);
}

void receive_hand(const motors::polarcoord &hand) {
	float alpha = -hand.angle;

	//if (hand.distance > 1) {
		printf("Received angle %f, modified to %f\n", hand.angle, alpha);
		Coord encoderInfo = movementController.moveTowardsTarget(alpha,
				hand.distance);
		Speed speed;
		//speed.W1 = (float) encoderInfo.y;
		//speed.W2 = (float) encoderInfo.x;
		speed.W1 = (float) 1.06;
		speed.W2 = (float) 1.67;
		printf("Speed left %f, right %f\n", speed.W1, speed.W2);
		pwm_pub.publish(speed);
		publish_marker_data();
	//}

}

void init() {
	movementController = MovementControl(0.05, 0.2, 360);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "FakeMotorsTest"); //Creates a node named "FakeMotorsTest"
	ros::NodeHandle n;
	enc_sub = n.subscribe("/motion/Encoders", 1000, receive_encoder); //when "/encoder" topic is revieved call recive_encoder function
	pwm_pub = n.advertise<Speed>("/motion/Speed", 1); //used to publish a topic that changes the motorspeed
	key_sub = n.subscribe("/human/keyboard", 1, receive_key); //when "/keyboard" topic is received, call back receive_key function
	hand_sub = n.subscribe("/hand/location", 1, receive_hand);

	// Settings for publishing to visualization
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",
			1);
	uint32_t shape = visualization_msgs::Marker::CUBE;

	ros::Rate loop_rate(100);

	init();

	struct timeval start, end;

	while (ros::ok()) {

		// Good old looping motor stuff
		loop_rate.sleep();
		//run(0, 0);
		ros::spinOnce();

	}
	return 0;
}
