/**
 * @file      motion.cpp
 * @brief     Main motion planning node
 * @author    Alexandre Silva <s1464657@sms.ed.ac.uk>
 * @date      2015-06-20
 */

#include "motion_planning/plan_object_detection.hpp"
#include "motion_planning/plan_pick.hpp"
#include "motion_planning/plan_place.hpp"
#include "motion_planning/plan_go_home.hpp"
#include "motion_planning/plan_listen_aoi.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "motion_planning");
  ros::NodeHandle nh("motion_planning");
  ROS_INFO("Running Motion Planning");

  PlanObjectDetectionAction detectionPlanner(nh, "plan_detection");
  PlanPickAction pickPlanner(nh, "plan_pick");
  PlanPlaceAction placePlanner(nh, "plan_place");
  PlanGoHomeAction goHomePlanner(nh, "plan_go_home");
  PlanListenAoiAction listenAoiPlanner(nh, "plan_listen_aoi");


  ros::spin();
  return 0;
}
