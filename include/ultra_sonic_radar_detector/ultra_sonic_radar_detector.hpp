/**
 * @file ultra_sonic_radar_detector.hpp
 * @author Mark Jin (mark@pixmoving.net)
 * @brief header file of ultra sonic radar detector
 * @version 0.1
 * @date 2022-12-22
 *
 * @copyright Copyright (c) 2022, Pixmoving
 * Rebuild The City With Autonomous Mobility
 * https://www.pixmoving.com
 *
 */
#ifndef __ULTRA_SONIC_RADAR_DETECTOR__HPP__
#define __ULTRA_SONIC_RADAR_DETECTOR__HPP__

#include <string>
#include <vector>

#include <ros/ros.h>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/PointCloud2.h>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

namespace ultra_sonic_radar_detector
{

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range,
    sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range, sensor_msgs::Range> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Sync;

/**
 * @brief param for ultra_sonic_radar_detector_node
 * 
 */
struct Param
{
  std::string output_frame; // output frame of concat pointcloud
  float cloud_radius_m; // radius of sphere pointcloud in meters
  float cloud_resolution_m; // resolution of pointcloud in meters
};

/**
 * @brief convert radar range msg to a pointcloud in the shape of sphere
 * 
 * @param range_ptr radar range msg ptr
 * @param radius radius of sphere in meters
 * @param resolution resolution of pointcloud in meters
 * @return sensor_msgs::PointCloud2 
 */
sensor_msgs::PointCloud2 rangeToPointCloud(
  const sensor_msgs::RangeConstPtr & range_ptr, const float & radius, const float & resolution);

class UltraSonicRadarDetector
{
private:
  Param param_;
  ros::NodeHandle nh_, pnh_;

  /**
   * @brief callback function of 8 radar topics
   * 
   * @param input_radar_0_msg 
   * @param input_radar_1_msg 
   * @param input_radar_2_msg 
   * @param input_radar_3_msg 
   * @param input_radar_4_msg 
   * @param input_radar_5_msg 
   * @param input_radar_6_msg 
   * @param input_radar_7_msg 
   */
  void radarsCallback(
    const sensor_msgs::RangeConstPtr & input_radar_0_msg,
    const sensor_msgs::RangeConstPtr & input_radar_1_msg,
    const sensor_msgs::RangeConstPtr & input_radar_2_msg,
    const sensor_msgs::RangeConstPtr & input_radar_3_msg,
    const sensor_msgs::RangeConstPtr & input_radar_4_msg,
    const sensor_msgs::RangeConstPtr & input_radar_5_msg,
    const sensor_msgs::RangeConstPtr & input_radar_6_msg,
    const sensor_msgs::RangeConstPtr & input_radar_7_msg);

  // tf2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // publisher
  ros::Publisher merged_pointcloud_pub_;

  // subscribers
  message_filters::Subscriber<sensor_msgs::Range> radar_0_sub_;
  message_filters::Subscriber<sensor_msgs::Range> radar_1_sub_;
  message_filters::Subscriber<sensor_msgs::Range> radar_2_sub_;
  message_filters::Subscriber<sensor_msgs::Range> radar_3_sub_;
  message_filters::Subscriber<sensor_msgs::Range> radar_4_sub_;
  message_filters::Subscriber<sensor_msgs::Range> radar_5_sub_;
  message_filters::Subscriber<sensor_msgs::Range> radar_6_sub_;
  message_filters::Subscriber<sensor_msgs::Range> radar_7_sub_;

  // synchronizer
  Sync sync_;

public:
  /**
   * @brief Construct a new Ultra Sonic Radar Detector object
   * 
   */
  UltraSonicRadarDetector();
  /**
   * @brief Destroy the Ultra Sonic Radar Detector object
   * 
   */
  ~UltraSonicRadarDetector();
  /**
   * @brief transform pointcloud from its our frame to output_frame
   * 
   * @param in1 input pointcloud
   * @param out output/transformed pointcloud
   */
  void transformPointCloud(
    const sensor_msgs::PointCloud2 & in1, sensor_msgs::PointCloud2 & out);
  /**
   * @brief concatenate 2 pointclouds which share the same frame
   * 
   * @param in1 input pointcloud 1
   * @param in2 input pointcloud 2
   * @param out output/concatenated pointcloud
   */
  void combineClouds(
    const sensor_msgs::PointCloud2 & in1, const sensor_msgs::PointCloud2 & in2,
    sensor_msgs::PointCloud2 & out);
};

}  // namespace ultra_sonic_radar_detector

#endif  // __ULTRA_SONIC_RADAR_DETECTOR__HPP__