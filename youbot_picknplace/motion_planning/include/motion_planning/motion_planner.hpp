#ifndef MOTION_PLANNER_HPP
#define MOTION_PLANNER_HPP



// ROS
#include <ros/ros.h>

//  PLUGIN
#include <pluginlib/class_loader.h>

//tf
#include <tf/transform_datatypes.h>

// Messages
#include "motion_planning_msgs/PlanMotion.h"

class MotionPlanner {
 public:
  explicit MotionPlanner(ros::NodeHandle* nh);
  ~MotionPlanner();
  bool planMotion(motion_planning_msgs::PlanMotion::Request& req,
        motion_planning_msgs::PlanMotion::Response& res);

 private:

  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_plan_motion_;

  // message fields
  geometry_msgs::PoseStamped pose;
  geometry_msgs::Quaternion quat;

};

#endif /* MOTION_PLANNER_HPP  */