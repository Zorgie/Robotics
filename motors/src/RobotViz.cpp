#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <differential_drive/Encoders.h>
#include "motors/polarcoord.h"

ros::Publisher marker_pub;
ros::Publisher hand_pub;
ros::Subscriber camera_sub;
ros::Subscriber enc_sub;

double x_robot_rviz_global = 0;
double y_robot_rviz_global = 0;
double theta_robot_rviz_global = 0;

double wheelRadius = 0.1;
double wheelDistance = 0.253;

void plotRobotState();

void updatePosition(const differential_drive::Encoders &msg){
	int leftEnc = msg.delta_encoder1;
	int rightEnc = msg.delta_encoder2;

	double leftSpeed = (leftEnc/360.0)*2*M_PI;		//speed [rad/s]
	double rightSpeed = (rightEnc/360.0)*2*M_PI;

	leftSpeed *= -1;

	double localX = (wheelRadius*leftSpeed)/2+(wheelRadius*rightSpeed)/2;
	double localY = 0;
	double localPhi = (wheelRadius*rightSpeed)/wheelDistance-(wheelRadius*leftSpeed)/wheelDistance;

	double globalDX = cos(theta_robot_rviz_global)*localX;
	double globalDY = sin(theta_robot_rviz_global)*localX;
	double globalDTheta = localPhi;

	x_robot_rviz_global += globalDX;
	y_robot_rviz_global += globalDY;
	theta_robot_rviz_global += globalDTheta;

	printf("left speed: %f,rightSpeedx: %f \n",leftSpeed,rightSpeed);
	//printf("glob dx: %f,loc dx: %f \n",globalDX,localX);
	//printf("robot_x: %f,robot_y: %f \n",x_robot_rviz_global,y_robot_rviz_global);

	plotRobotState();
}

void plotHand(const motors::polarcoord &hand){

	  visualization_msgs::Marker marker;
	  marker.header.frame_id = "/robot_viz_frame";
	  marker.header.stamp = ros::Time::now();
	  marker.ns = "robot_viz_markers";
	  marker.id = 0;
	  marker.type = visualization_msgs::Marker::CUBE;
	  marker.action = visualization_msgs::Marker::ADD;

	  //marker.pose.position.x = hand.distance*cos(hand.angle);
	  //marker.pose.position.y = hand.distance*sin(hand.angle);
	  marker.pose.position.x = x_robot_rviz_global + cos(theta_robot_rviz_global)*hand.distance*cos(hand.angle) - sin(theta_robot_rviz_global)*hand.distance*sin(hand.angle);
	  marker.pose.position.y = y_robot_rviz_global + sin(theta_robot_rviz_global)*hand.distance*cos(hand.angle) + cos(theta_robot_rviz_global)*hand.distance*sin(hand.angle);;




	  marker.pose.orientation.x = 0;
	  marker.pose.orientation.y = 0;
	  marker.pose.orientation.z = 0;
	  marker.pose.orientation.w = 1.0;

	  marker.scale.x = 0.05;
	  marker.scale.y = 0.05;
	  marker.scale.z = 0.05;

	  marker.color.r = 0.0f;
	  marker.color.g = 1.0f;
	  marker.color.b = 0.0f;
	  marker.color.a = 1.0f;

	  marker.lifetime = ros::Duration();
	  hand_pub.publish(marker);
}


//receive whatever msg we get
void plotRobotState(){

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/robot_viz_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "robot_viz_markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = x_robot_rviz_global;
  marker.pose.position.y = y_robot_rviz_global;
  marker.pose.position.z = 0;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = sin(theta_robot_rviz_global/2);
  marker.pose.orientation.w = cos(theta_robot_rviz_global/2);;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.lifetime = ros::Duration();
  marker_pub.publish(marker);
}


int main(int argc,char **argv){
  ros::init(argc,argv,"RobotViz");
  ros::NodeHandle n;
  ros::Rate loop_rate(40);

  //todo: subscribe to node publishing robot coords
  //ros::Subscriber sub = n.subscribe("topicname",1,plotRobotState);

  marker_pub = n.advertise<visualization_msgs::Marker>("robot_viz_markers",1);
  hand_pub = n.advertise<visualization_msgs::Marker>("hand_viz_markers",1);
  camera_sub = n.subscribe("/hand/location", 1, plotHand);
  enc_sub = n.subscribe("/motion/Encoders", 1, updatePosition);

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
