/*
 * Mapper.h
 *
 *  Created on: Nov 26, 2013
 *      Author: Lucas Taubert
 */

#ifndef MAPPER_H_
#define MAPPER_H_

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <highgui.h>
#include <cv.h>
#include <irsensors/floatarray.h>
#include <mapping/robot_pose.h>
#include "NavMap.h"

// Navigation messages etc.
#include <navigation/movement_state.h>
#include <navigation/path_request.h>
#include <navigation/path_result.h>
#include <navigation/RobotActions.h>
#include "tutorialROSOpenCV/evidence.h"
#include "std_msgs/String.h"


class Mapper {
private:
	NavMap nav;
	ros::NodeHandle nh;
	ros::Subscriber irSub;
	ros::Subscriber poseSub;
	ros::Subscriber camSub;
	ros::Subscriber movementSub;
	ros::Subscriber pathRequestSub;
	ros::Subscriber objectSub;

	ros::Publisher posePub;
	ros::Publisher pathResultPub;
	ros::Publisher speakerPub;

	mapping::robot_pose currentPose;
	bool poseInit;

	int findPath;

	bool goneHome;
	bool acceptNode;

	char* WINDOW;

	bool useGui;

	bool validIR(double r1, double r2);
public:
	Mapper(bool gui);
	virtual ~Mapper();
	void irCallback(const irsensors::floatarray& msg);
	void depthCallback(const sensor_msgs::PointCloud2& pcloud);
	void poseCallback(const mapping::robot_pose& p);
	void pathRequestCallback(const navigation::path_request& p);
	void addObject(double x, double y);
	void movementCommandCallback(const navigation::movement_state& state);
	mapping::robot_pose calibratePos(irsensors::floatarray currentIR);

	void pathResultCallback(vector<Edge> path);
	void objectDetectedCallback(const tutorialROSOpenCV::evidence &msg);
};

#endif /* MAPPER_H_ */
