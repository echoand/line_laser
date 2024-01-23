#pragma once
#include "yaml-cpp/yaml.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class Line {
 public:
  YAML::Node config;
  struct Output {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_last;
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall_last;
  };

  pcl::PointCloud<pcl::PointXYZ>::Ptr statisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                                const YAML::Node &config);
  void fitLine(const pcl::PointCloud<pcl::PointXYZ> &point_cloud, double &gradient, double &intercept);
  float calculateDistanceToPlane(const pcl::PointXYZ &point, const double &a, const double &b);
  Output Callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr &init_msg, const YAML::Node &config);
  Output processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &scan_msg);



};