#include "line_lasder.h"
#include "rosbridge.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pub1;
ros::Publisher pub2;
sensor_msgs::PointCloud ground_output;
sensor_msgs::PointCloud wall_output;
using namespace std;
void msgcallback(const sensor_msgs::LaserScan::ConstPtr &input_msg) {
  Line line;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_points(new pcl::PointCloud<pcl::PointXYZ>);//转换成pcl格式的原始点云
  rostopcl(input_msg, pcl_points);
  Line::Output result = line.processPointCloud(pcl_points);

  pcltoros(result.ground_last, ground_output);
  pcltoros(result.wall_last, wall_output);
  cout << "地面滤波后点云的数量" << ground_output.points.size() << endl;
  ground_output.header.frame_id = input_msg->header.frame_id;
  pub1.publish(ground_output);

  cout << "墙面滤波后点云的数量" << wall_output.points.size() << endl;
  wall_output.header.frame_id = input_msg->header.frame_id;
  pub2.publish(wall_output);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "line_lidar");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, msgcallback);
  pub1 = nh.advertise<sensor_msgs::PointCloud>("/ground", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud>("/wall", 1);

  ros::spin();

  return 0;

}