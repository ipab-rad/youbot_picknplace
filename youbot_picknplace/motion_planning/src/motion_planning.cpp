/**
 * @file      motion.cpp
 * @brief     Main motion planning node
 * @author    Alexandre Silva <s1464657@sms.ed.ac.uk>
 * @date      2015-06-20
 */

 #include "motion_planning/motion_planner.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_planning");
  ros::NodeHandle nh("motion_planning");
  ROS_INFO("Running Motion Planning");

  MotionPlanner motionplanner(&nh);


  ros::spin();
  return 0;
}
