/**
 * @file      motion.cpp
 * @brief     Main motion node
 * @author    Alexandre Silva <s1464657@sms.ed.ac.uk>
 * @date      2015-06-20
 */

#include "motion/move_to_position.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion");
  ros::NodeHandle nh("motion");
  ROS_INFO("Running Motion");
  MoveToPositionAction movetoposition(nh, "move_to_position");


  ros::spin();

  ros::shutdown();

  return 0;
}
