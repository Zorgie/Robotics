/*
 * DepthReader.cpp
 *
 *  Created on: Nov 14, 2013
 *      Author: robo
 */

#include "DepthReader.h"

DepthReader::DepthReader() {
	// TODO Auto-generated constructor stub

}

DepthReader::~DepthReader() {
	// TODO Auto-generated destructor stub
}

void DepthReader::process(const sensor_msgs::PointCloud2ConstPtr& input){

		sensor_msgs::PointCloud2 cloud_filtered;

		// Perform the actual filtering
		pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
		sor.setInputCloud(input);
		sor.setLeafSize(0.01, 0.01, 0.01);
		sor.filter(cloud_filtered);

		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::fromROSMsg(cloud_filtered, cloud);

		polarcoord polar;

		if(cloud.points.size() == 0){
			printf("Invalid coords\n");
			polar.angle = 0;
			polar.distance = 0;
			pub.publish(polar);
			return;
		}

		float zMin = 0;
		int iMin=0;
		float x = 0, y = 0;

		zMin = cloud.points[0].z;
		for (int i = 0; i < cloud.points.size(); i++) {
			float point = cloud.points[i].z;
			if (valid_coords(cloud.points[i].x, cloud.points[i].y, point)) {
				if (zMin > point) {
					zMin = point;
					x = cloud.points[i].x;
					y = cloud.points[i].y;
					iMin=i;

				}
			}
		}
		printf("Min: %f, X: %f, Y: %f\n", zMin, x, y);
		printf("1: %f 2: %f 3: %f 4: %f \n",cloud.points[iMin].data[0],cloud.points[iMin].data[1],cloud.points[iMin].data[2],cloud.points[iMin].data[3]);

		// Publish the data
		//pub.publish(cloud_filtered);
	}
}
