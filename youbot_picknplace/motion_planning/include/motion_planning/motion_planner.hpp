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

// move it libs
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>


class MotionPlanner {
 public:
  explicit MotionPlanner(ros::NodeHandle* nh);
  ~MotionPlanner();

  planning_interface::PlannerManagerPtr getPlannerInstance();
  void setRobotModel();
  planning_scene::PlanningScenePtr getPlanningScene();

  bool planMotion(motion_planning_msgs::PlanMotion::Request& req,
                      motion_planning_msgs::PlanMotion::Response& res);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::ServiceServer srv_plan_motion_;

  // robot
  robot_model::RobotModelPtr robot_model;


  // message fields
  geometry_msgs::PoseStamped pose;
  geometry_msgs::Quaternion quat;
  planning_interface::MotionPlanRequest request;
  planning_interface::MotionPlanResponse response;

};

#endif /* MOTION_PLANNER_HPP  */