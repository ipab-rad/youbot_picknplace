/**
 * @file      sensing.cpp
 * @brief     Main sensing node
 * @author    Alexandre Silva <s1464657@sms.ed.ac.uk>
 * @date      2015-06-20
 */

#include "sensing/object_detection.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensing");
  ros::NodeHandle nh("~");
  ROS_INFO("Running Sensing");

  DetectObjectAction obj_detection(nh, "object_detection");

  ros::spin();
  return 0;
}
