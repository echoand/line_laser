//
// Created by bz on 24-1-23.
//

#ifndef LINELIB__ROSBRIDGE_H_
#define LINELIB__ROSBRIDGE_H_
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/PointCloud.h"
void rostopcl(const sensor_msgs::LaserScan::ConstPtr &ros_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg);
void pcltoros(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_points, sensor_msgs::PointCloud &output);

#endif //LINELIB__ROSBRIDGE_H_
