#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include "RobotPosition.h"

#include <cmath>
#include <vector>

static movement::robot_pose global_current_robot_pose;
static ros::Subscriber robot_pose_sub;

static const int UPDATE_RATE = 50;

void robot_pose_update(const movement::robot_pose &msg) {
	global_current_robot_pose = msg;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  robot_pose_sub = n.subscribe("/robot_pose", 1, robot_pose_update);

  std::vector<movement::robot_pose> pose_history;

  ros::Rate r(UPDATE_RATE);

  float f = 0.0;
  while (ros::ok())
  {
	ros::spinOnce();

    visualization_msgs::Marker points;
    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Create the vertices for the points and lines
//    for (uint32_t i = 0; i < 100; ++i)
//    {
//      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
//      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
//
//      geometry_msgs::Point p;
//      p.x = (int32_t)i - 50;
//      p.y = y;
//      p.z = z;
//
//      points.points.push_back(p);
//
//   }

    pose_history.push_back(global_current_robot_pose);

    for (std::vector<movement::robot_pose>::iterator it = pose_history.begin();
        it != pose_history.end(); ++it){

        geometry_msgs::Point p;
        p.x = it->x;
        p.y = it->y;
        p.z = 0.0;

        points.points.push_back(p);
    }

    marker_pub.publish(points);

    r.sleep();

    f += 0.04;
  }
}
