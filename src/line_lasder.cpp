#include <iostream>
#include "line_lasder.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "pcl_conversions/pcl_conversions.h"
#include "laser_geometry/laser_geometry.h"
#include "pcl_conversions/pcl_conversions.h"

using namespace std;
using namespace Eigen;
using namespace std::chrono;

pcl::PointCloud<pcl::PointXYZ>::Ptr Line::statisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                                                                    const YAML::Node &config) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  int adjacent_points = config["adjacent_points"].as<int>();//20
  float tolerance = config["tolerance"].as<float>();//0.07
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(input_cloud);
  sor.setMeanK(adjacent_points);//进行统计时考虑查询点邻近点数
  sor.setStddevMulThresh(tolerance);//值越大对噪点的容忍度越高
  sor.filter(*filtered_cloud);

  return filtered_cloud;
}
void Line::fitLine(const pcl::PointCloud<pcl::PointXYZ> &point_cloud, double &gradient, double &intercept) {
  // 检查点云的大小，确保其大于等于2
  if (point_cloud.size() < 2) {
    std::cerr << "点云数量小于2,拟合失败" << std::endl;
    return;
  }

  Eigen::MatrixXd A(point_cloud.size(), 2);

  // 构建矩阵 A
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    A(i, 0) = point_cloud[i].x;
    A(i, 1) = 1;
  }

  // 构建向量 b
  Eigen::VectorXd b(point_cloud.size());
  for (size_t i = 0; i < point_cloud.size(); ++i) {
    b(i) = point_cloud[i].y;
  }

  // 使用 SVD 方法求解最小二乘问题
  Eigen::VectorXd solution = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  // 获取斜率和截距
  gradient = solution(0);  // 直线的斜率
  intercept = solution(1); // 直线的截距
}
float Line::calculateDistanceToPlane(const pcl::PointXYZ &point, const double &a, const double &b) {
  // 计算点到平面的距离
  float distance = std::abs(point.y - a * point.x - b) / std::sqrt(1 + a * a);

  return distance;
}

Line::Output Line::Callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_points, const YAML::Node &config) {

  cout << "转成pcl原始点云的数量:" << pcl_points->points.size() << endl;
  auto begin = steady_clock::now();//获取系统启动后到现在的时间点

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);//筛选后的地面点
  pcl::PointCloud<pcl::PointXYZ>::Ptr wall_points(new pcl::PointCloud<pcl::PointXYZ>);//筛选后的墙面点

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_filtered(new pcl::PointCloud<pcl::PointXYZ>);//滤波后的地面点
  pcl::PointCloud<pcl::PointXYZ>::Ptr wall_filtered(new pcl::PointCloud<pcl::PointXYZ>);//滤波后的墙面点

//2.筛选点


  for (int i = 1; i < pcl_points->points.size(); ++i) {
    if (i < 1) {
      continue;
    }
    float outside = config["outside"].as<float>();//0.0015
    float within = config["within"].as<float>();//0.0009
    // 根据 y 坐标间距逐渐增大以及 x 坐标间距相差不大的特点，筛选地面点和墙上的点
    if (abs(pcl_points->points[i].x - pcl_points->points[i - 1].x) > outside
        || (abs(pcl_points->points[i].y - pcl_points->points[i - 1].y) < within)) {
      ground_points->points.push_back(pcl_points->points[i]);
    } else if (abs(pcl_points->points[i].y - pcl_points->points[i - 1].y) > outside
        || (abs(pcl_points->points[i].x - pcl_points->points[i - 1].x) < within)) {
      wall_points->points.push_back(pcl_points->points[i]);
    }

  }

  cout << "筛选地面点数量: " << ground_points->points.size() << endl;
  cout << "筛选墙面点数量: " << wall_points->points.size() << endl;

//3.滤波

  ground_filtered = statisticalOutlierRemoval(ground_points, config);
  wall_filtered = statisticalOutlierRemoval(wall_points, config);

//4.进行拟合 得到两条线上的系数
  double g_a, g_b, w_a, w_b;
  fitLine(*ground_filtered, g_a, g_b);
  fitLine(*wall_filtered, w_a, w_b);

//5.根据距离对原始点云进行筛选
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_last(new pcl::PointCloud<pcl::PointXYZ>);//地面点滤波数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr wall_last(new pcl::PointCloud<pcl::PointXYZ>);//墙面点滤波数据
  for (int i = 1; i < pcl_points->points.size(); ++i) {
    //算原始点到两条线上的距离
    float dist = config["dist"].as<float>();//0.007
    float dist_g = calculateDistanceToPlane(pcl_points->points[i], g_a, g_b);
    float dist_w = calculateDistanceToPlane(pcl_points->points[i], w_a, w_b);
    if (dist_g < dist) {
      ground_last->push_back(pcl_points->points[i]);
    } else if (dist_w < dist) {
      wall_last->push_back(pcl_points->points[i]);// #include <boost/thread/thread.hpp>


    }

  }
  auto end = steady_clock::now();
  auto time = (end - begin).count() / 1e9;//返回时间差，单位纳秒
  cout << "耗时" << time << "秒" << std::endl;

  cout << "地面滤波后点云的数量" << ground_last->points.size() << endl;
  cout << "墙面滤波后点云的数量" << wall_last->points.size() << endl;
  Output result;
// 将计算得到的 ground_last、wall_last 和时间信息存入 result 中
  result.ground_last = ground_last;
  result.wall_last = wall_last;

  return result;
}


Line::Output Line::processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr&scan_msg) {
  config = YAML::LoadFile("../param/config.yaml");
  Output result = Callback(scan_msg, config);
  return result;
}