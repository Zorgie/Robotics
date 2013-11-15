/*
 * DepthReader.h
 *
 *  Created on: Nov 14, 2013
 *      Author: robo
 */

#ifndef DEPTHREADER_H_
#define DEPTHREADER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <math.h>

class DepthReader {
public:
	DepthReader();
	virtual ~DepthReader();
	void process(const sensor_msgs::PointCloud2ConstPtr& input);
};

#endif /* DEPTHREADER_H_ */
