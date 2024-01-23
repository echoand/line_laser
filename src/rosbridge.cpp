//
// Created by bz on 24-1-23.
//

#include <pcl/point_cloud.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rosbridge.h"

void rostopcl(const sensor_msgs::LaserScan::ConstPtr &ros_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_msg) {
//把LaserScan格式转换成PointCloud2格式
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud2 cloud;
  projector.projectLaser(*ros_msg, cloud);
//ros转pcl
  pcl::fromROSMsg(cloud, *pcl_msg);

}
void pcltoros(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcl, sensor_msgs::PointCloud &output) {
  output.points.clear();
  for (size_t i = 0; i < input_pcl->width * input_pcl->height; ++i) {
    geometry_msgs::Point32 point;
    point.x = input_pcl->points[i].x;
    point.y = input_pcl->points[i].y;
    point.z = input_pcl->points[i].z;
    output.points.push_back(point);
  }
}