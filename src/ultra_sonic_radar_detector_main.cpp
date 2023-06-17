/**
 * @file ultra_sonic_radar_detector_main.cpp
 * @author Mark Jin (mark@pixmoving.net)
 * @brief main function of ultra sonic radar detector
 * @version 0.1
 * @date 2022-12-22
 * 
 * @copyright Copyright (c) 2022, Pixmoving
 * Rebuild The City With Autonomous Mobility
 * https://www.pixmoving.com
 * 
 */

#include "ultra_sonic_radar_detector/ultra_sonic_radar_detector.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "ultra_sonic_radar_detector_node");
  ultra_sonic_radar_detector::UltraSonicRadarDetector node;
  ros::spin();
  return 0;
}