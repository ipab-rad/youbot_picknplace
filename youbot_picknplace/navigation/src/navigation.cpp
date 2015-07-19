/**
 * @file      navigation.cpp
 * @brief     Main navigation node
 * @author    Alexandre Silva <s1464657@sms.ed.ac.uk>
 * @date      2015-06-20
 */

#include "navigation/move_to_position.hpp"


int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation");
  ros::NodeHandle nh("navigation");

  ROS_INFO("Running Navigation");
  MoveToPositionAction moveToPosition(nh, "move_to_position");


  ros::spin();
  return 0;
}
