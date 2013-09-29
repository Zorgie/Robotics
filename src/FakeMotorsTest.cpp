#include "ros/ros.h"
#include <std_msgs/Int32.h>

#include "differential_drive/KeyEvent.h"
#include <differential_drive/PWM.h>
#include <differential_drive/Encoders.h>

#include <visualization_msgs/Marker.h>

#include <string.h>
#include <math.h>
#include <SDL/SDL.h>

using namespace differential_drive;
ros::Subscriber enc_sub;
ros::Subscriber key_sub;
ros::Publisher pwm_pub;
ros::Publisher marker_pub;
int r_movement;
int l_movement;
int r_movespeed;
int l_movespeed;
double x_orientation;
double y_orientation;
double direction;
double x_pos;
double y_pos;
static const double PI = 3.14159265359;

void publish_marker_data() {
	// Visualization stuff
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/my_frame";
	marker.header.stamp = ros::Time::now();
	// Sets namespace and ID
	marker.ns = "basic_shapes";
	marker.id = 0;
	// Sets the type of visual to cube; and add as action. Add also works as update.
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;

	// Position
	marker.pose.position.x = x_pos;
	marker.pose.position.y = y_pos;
	marker.pose.position.z = 0;

    marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = sin(direction * PI/360);
	marker.pose.orientation.w = cos(direction * PI/360);
	PWM pwm;
	// Green180/PI
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;
	// Normal scale
	marker.scale.x = 10.0;
	marker.scale.y = 10.0;
	marker.scale.z = 10.0;
	// I WANNA LIVE FOREVER
	marker.lifetime = ros::Duration();
	marker_pub.publish(marker);
}

void receive_key(const KeyEvent::ConstPtr &msg) //Taken from KeyboardControl.cpp
		{
	switch (msg->sym) {
	case SDLK_UP:
		if (msg->pressed) {
			r_movement = 100;
			l_movement = 100;
		}
		break;
	case SDLK_DOWN:
		if (msg->pressed) {
			r_movement = -100;
			l_movement = -100;
		}
		break;
	case SDLK_LEFT:
		if (msg->pressed) {
			r_movement = 100;
			l_movement = -100;
		}
		break;
	case SDLK_RIGHT:
		if (msg->pressed) {
			r_movement = -100;
			l_movement = 100;
		}
		break;
	case SDLK_SPACE:
		if (msg->pressed) {
			l_movement = 0;
			r_movement = 0;
		}
	}
}

void calculate_orientation() {
	while (direction > 360)
		direction -= 360;
	while (direction < 0)
		direction += 360;
	x_orientation = cos(direction * PI / 180);
	y_orientation = sin(direction * PI / 180);
}

//Callback function for the "/encoder" topic. Prints the enconder value and changes the speed accordingly.
void receive_encoder(const Encoders::ConstPtr &msg) {
	static ros::Time t_start = ros::Time::now();
	int right = msg->delta_encoder2;
	int left = msg->delta_encoder1;
	r_movement -= right;
	l_movement -= left;
	direction += (right - left);
	calculate_orientation();

	x_pos += x_orientation * ((double) (right + left) / 2);
	y_pos += y_orientation * ((double) (right + left) / 2);

	int timestamp = msg->timestamp;
	printf("%d:got encoder L:%d , false R:%d\n", timestamp, left, right);
	// Handles the movement, lol.
	PWM pwm;
	if (r_movement > 5) {
		pwm.PWM2 = r_movespeed;
	} else if (r_movement < -5) {
		pwm.PWM2 = -r_movespeed;
	}
	if (l_movement > 5) {
		pwm.PWM1 = l_movespeed;
	} else if (l_movement < -5) {
		pwm.PWM1 = -l_movespeed;
	}
	pwm.header.stamp = ros::Time::now();
	pwm_pub.publish(pwm);
	publish_marker_data();
}

// Function to start moving foward.
void set_move_forward(int encoder_units) {
	r_movement += encoder_units;
	l_movement += encoder_units;
}

void init() {
	r_movement = 0;
	l_movement = 0;
	r_movespeed = 100;
	l_movespeed = 100;
	x_pos = 0;
	y_pos = 0;
	x_orientation = 0;
	y_orientation = 0;
	direction = 0;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "FakeMotorsTest"); //Creates a node named "FakeMotorsTest"
	ros::NodeHandle n;
	enc_sub = n.subscribe("/motors/encoders", 1000, receive_encoder); //when "/encoder" topic is revieved call recive_encoder function
	pwm_pub = n.advertise < PWM > ("/motors/pwm", 100000); //used to publish a topic that changes the motorspeed
	key_sub = n.subscribe("/human/keyboard", 1000, receive_key); //when "/keyboard" topic is received, call back receive_key function

	// Settings for publishing to visualization
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	printf("trying to publish.");
	uint32_t shape = visualization_msgs::Marker::CUBE;

	ros::Rate loop_rate(100);
	//The loop randomly changes the intervall with wich the "/encoder" topic is published.

	init();

	struct timeval start, end;

	while (ros::ok()) {

		// Good old looping motor stuff
		loop_rate.sleep();
		ros::spinOnce();

	}
	return 0;
}
