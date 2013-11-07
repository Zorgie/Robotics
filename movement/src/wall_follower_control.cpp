#include "ros/ros.h"
#include <differential_drive/Encoders.h>
#include <differential_drive/PWM.h>
#include "movement/wheel_speed.h"
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <irsensors/floatarray.h>

#include "WallFollow.h"

using namespace differential_drive;

//defines the speed of the Control Loop (in Hz)
const int UPDATE_RATE = 100; // maybe this value can be changed for the high level control, kanske 10?  ARE WE GOING TO HAVE PROBLEMS WITH HAVING THIS VARIABLE DECLARED IN SEVERAL PROGRAMS?

ros::Subscriber ir_sub; // Subscriber to the ir post-processing readings;
ros::Publisher desired_speed_pub; // Publisher of the desired speed to the Low Level Controller;

irsensors::floatarray ir_readings_processed_global; // This variable stores the last read ir values;

void ir_readings_update(const irsensors::floatarray &msg) { // movement::wheel_speed
	// Whenever we get a callback from the ir readings, we must update our current ir readings;
	ir_readings_processed_global = msg; // Save the most recent values;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "WallFollowerController"); // Name of the node is WallFollowerController
	ros::NodeHandle n;

	n.setParam("/param_gain", 5.0);

	ir_sub = n.subscribe("/sensors/transformed/ADC", 1, ir_readings_update); // Subscribing to the processed ir values topic.

	ros::Rate loop_rate(UPDATE_RATE);

	// Creates a WallFollower object.
	WallFollow wf;
	// Runs the initiation method (initializes the variable) on the WallFoldlower object.
	wf.init();
	desired_speed_pub = n.advertise<movement::wheel_speed>("/desired_speed", 1); // We are Publishing a topic that changes the desired wheel speeds.

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
		// Runs the step method of the wallfollower object, which remembers the state through fields (variables).
		double param_gain;
		if (n.getParam("/param_gain", param_gain)) {
			//printf("CHANGE\n");
		}
		movement::wheel_speed desired_speed = wf.step(param_gain, ir_readings_processed_global);
		desired_speed_pub.publish(desired_speed);
	}
}
