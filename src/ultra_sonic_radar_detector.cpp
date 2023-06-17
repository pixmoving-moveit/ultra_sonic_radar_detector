/**
 * @file ultra_sonic_radar_detector.cpp
 * @author Mark Jin (mark@pixmoving.net)
 * @brief implemention of ultra sonic radar detector
 * @version 0.1
 * @date 2022-12-22
 * 
 * @copyright Copyright (c) 2022, Pixmoving
 * Rebuild The City With Autonomous Mobility
 * https://www.pixmoving.com
 * 
 */

#include "ultra_sonic_radar_detector/ultra_sonic_radar_detector.hpp"

namespace ultra_sonic_radar_detector
{

sensor_msgs::PointCloud2 rangeToPointCloud(
  const sensor_msgs::RangeConstPtr & range_ptr, const float & radius, const float & resolution)
{
  sensor_msgs::PointCloud2 output_pointcloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_ptr->width = 1000;
    cloud_ptr->height = 1000;
    cloud_ptr->is_dense = false;
    cloud_ptr->points.reserve(cloud_ptr->width * cloud_ptr->height);
  if (range_ptr->range > range_ptr->max_range || range_ptr->range < range_ptr->min_range ||
      radius == 0.0) {
    cloud_ptr->resize(cloud_ptr->points.size());
    pcl::toROSMsg(*cloud_ptr, output_pointcloud);
    output_pointcloud.header = range_ptr->header;
    return output_pointcloud;
  }
  for (float y = -radius; y <= radius; y += resolution) {
    for (float x = -radius; x <= radius; x += resolution) {
      for (float z = -radius; z <= radius; z += resolution) {
        if (x * x + y * y + z * z <= radius * radius) {
          pcl::PointXYZ point;
          point.x = x+range_ptr->range;
          point.y = y;
          point.z = z;
          cloud_ptr->points.push_back(point);
        }
      }
    }
  }
  cloud_ptr->resize(cloud_ptr->points.size());
  pcl::toROSMsg(*cloud_ptr, output_pointcloud);
  output_pointcloud.header = range_ptr->header;
  return output_pointcloud;
}

void UltraSonicRadarDetector::transformPointCloud(
  const sensor_msgs::PointCloud2 & in, sensor_msgs::PointCloud2 & out)
{
  if (param_.output_frame != in.header.frame_id) {
    try {
      geometry_msgs::TransformStamped transformStamped =
        tf_buffer_.lookupTransform(param_.output_frame, in.header.frame_id, ros::Time(0));
      // std::cout << transformStamped << std::endl;
      tf2::doTransform(in, out, transformStamped);
    } catch (tf2::TransformException & ex) {
      ROS_WARN(
        "[UltraSonicRadarDetector::transformPointCloud] Error converting from %s to %s, %s",
        in.header.frame_id.c_str(), param_.output_frame.c_str(), ex.what());
      return;
    }
  } else {
    out = in;
  }
}

void UltraSonicRadarDetector::combineClouds(
  const sensor_msgs::PointCloud2 & in1, const sensor_msgs::PointCloud2 & in2,
  sensor_msgs::PointCloud2 & out)
{
  pcl::concatenatePointCloud(in1, in2, out);
  out.header.stamp = std::min(in1.header.stamp, in2.header.stamp);
}

UltraSonicRadarDetector::UltraSonicRadarDetector()
: nh_(""),
  pnh_("~"),
  tf_listener_(tf_buffer_),
  radar_0_sub_(pnh_, "input/radar_0", 1),
  radar_1_sub_(pnh_, "input/radar_1", 1),
  radar_2_sub_(pnh_, "input/radar_2", 1),
  radar_3_sub_(pnh_, "input/radar_3", 1),
  radar_4_sub_(pnh_, "input/radar_4", 1),
  radar_5_sub_(pnh_, "input/radar_5", 1),
  radar_6_sub_(pnh_, "input/radar_6", 1),
  radar_7_sub_(pnh_, "input/radar_7", 1),
  sync_(
    SyncPolicy(5), radar_0_sub_, radar_1_sub_, radar_2_sub_, radar_3_sub_, radar_4_sub_,
    radar_5_sub_, radar_6_sub_, radar_7_sub_)
{
  pnh_.getParam("output_frame", param_.output_frame);
  pnh_.getParam("cloud_radius_m", param_.cloud_radius_m);
  pnh_.getParam("cloud_resolution_m", param_.cloud_resolution_m);
  sync_.registerCallback(
    boost::bind(&UltraSonicRadarDetector::radarsCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
  merged_pointcloud_pub_ =
    pnh_.advertise<sensor_msgs::PointCloud2>("output/pointcloud", 10);
}

UltraSonicRadarDetector::~UltraSonicRadarDetector()
{
  
}

void UltraSonicRadarDetector::radarsCallback(
  const sensor_msgs::RangeConstPtr & input_radar_0_msg,
  const sensor_msgs::RangeConstPtr & input_radar_1_msg,
  const sensor_msgs::RangeConstPtr & input_radar_2_msg,
  const sensor_msgs::RangeConstPtr & input_radar_3_msg,
  const sensor_msgs::RangeConstPtr & input_radar_4_msg,
  const sensor_msgs::RangeConstPtr & input_radar_5_msg,
  const sensor_msgs::RangeConstPtr & input_radar_6_msg,
  const sensor_msgs::RangeConstPtr & input_radar_7_msg)
{
  // Create 8 point clouds
  sensor_msgs::PointCloud2 cloud_0 =
    rangeToPointCloud(input_radar_0_msg, param_.cloud_radius_m, param_.cloud_resolution_m);
  sensor_msgs::PointCloud2 cloud_1 =
    rangeToPointCloud(input_radar_1_msg, param_.cloud_radius_m, param_.cloud_resolution_m);
  sensor_msgs::PointCloud2 cloud_2 =
    rangeToPointCloud(input_radar_2_msg, param_.cloud_radius_m, param_.cloud_resolution_m);
  sensor_msgs::PointCloud2 cloud_3 =
    rangeToPointCloud(input_radar_3_msg, param_.cloud_radius_m, param_.cloud_resolution_m);
  sensor_msgs::PointCloud2 cloud_4 =
    rangeToPointCloud(input_radar_4_msg, param_.cloud_radius_m, param_.cloud_resolution_m);
  sensor_msgs::PointCloud2 cloud_5 =
    rangeToPointCloud(input_radar_5_msg, param_.cloud_radius_m, param_.cloud_resolution_m);
  sensor_msgs::PointCloud2 cloud_6 =
    rangeToPointCloud(input_radar_6_msg, param_.cloud_radius_m, param_.cloud_resolution_m);
  sensor_msgs::PointCloud2 cloud_7 =
    rangeToPointCloud(input_radar_7_msg, param_.cloud_radius_m, param_.cloud_resolution_m);

  sensor_msgs::PointCloud2 cloud_tranformed_0;
  sensor_msgs::PointCloud2 cloud_tranformed_1;
  sensor_msgs::PointCloud2 cloud_tranformed_2;
  sensor_msgs::PointCloud2 cloud_tranformed_3;
  sensor_msgs::PointCloud2 cloud_tranformed_4;
  sensor_msgs::PointCloud2 cloud_tranformed_5;
  sensor_msgs::PointCloud2 cloud_tranformed_6;
  sensor_msgs::PointCloud2 cloud_tranformed_7;
  transformPointCloud(cloud_0, cloud_tranformed_0);
  transformPointCloud(cloud_1, cloud_tranformed_1);
  transformPointCloud(cloud_2, cloud_tranformed_2);
  transformPointCloud(cloud_3, cloud_tranformed_3);
  transformPointCloud(cloud_4, cloud_tranformed_4);
  transformPointCloud(cloud_5, cloud_tranformed_5);
  transformPointCloud(cloud_6, cloud_tranformed_6);
  transformPointCloud(cloud_7, cloud_tranformed_7);
  // concatenate pointclouds
  sensor_msgs::PointCloud2 output_clouds_0;
  sensor_msgs::PointCloud2 output_clouds_1;
  sensor_msgs::PointCloud2 output_clouds_2;
  sensor_msgs::PointCloud2 output_clouds_3;
  sensor_msgs::PointCloud2 output_clouds_4;
  sensor_msgs::PointCloud2 output_clouds_5;
  sensor_msgs::PointCloud2 output_clouds_6;
  combineClouds(cloud_tranformed_0, cloud_tranformed_1, output_clouds_0);
  combineClouds(output_clouds_0, cloud_tranformed_2, output_clouds_1);
  combineClouds(output_clouds_1, cloud_tranformed_3, output_clouds_2);
  combineClouds(output_clouds_2, cloud_tranformed_4, output_clouds_3);
  combineClouds(output_clouds_3, cloud_tranformed_5, output_clouds_4);
  combineClouds(output_clouds_4, cloud_tranformed_6, output_clouds_5);
  combineClouds(output_clouds_5, cloud_tranformed_7, output_clouds_6);
  output_clouds_6.header.frame_id = param_.output_frame;
  output_clouds_6.header.stamp = ros::Time::now();
  merged_pointcloud_pub_.publish(output_clouds_6);
}

}  // namespace ultra_sonic_radar_detector
