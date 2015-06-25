/**
 * @file      monitor.hpp
 * @brief     Monitor class, leds and audio are used for external debugging
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-06-17
 * @copyright (MIT) 2015 Edinferno
 */

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
};

#endif /* MOTION_PLANNER_HPP */