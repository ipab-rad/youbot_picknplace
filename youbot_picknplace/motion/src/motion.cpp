/**
 * @file      motion.cpp
 * @brief     Main motion node
 * @author    Alexandre Silva <s1464657@sms.ed.ac.uk>
 * @date      2015-06-20
 */

#include "motion/move_to_pose.hpp"
#include "motion/move_to_posture.hpp"
// #include "motion/move_gripper.hpp"


int main(int argc, char** argv) {
  ros::init(argc, argv, "motion");
  ros::NodeHandle nh("motion");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Running Motion");
  MoveToPoseAction movetopose(nh, "move_to_pose");
  MoveToPostureAction movetoposture(nh, "move_to_posture");
  // MoveGripperAction movegripper(nh, "move_gripper");


  ros::spin();
  return 0;
}
