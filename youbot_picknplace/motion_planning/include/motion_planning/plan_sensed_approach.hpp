#ifndef PLAN_SENSED_APPROACH_SERVER_HPP
#define PLAN_SENSED_APPROACH_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
// Messages
#include "sensing_msgs/DetectObjectAction.h"
#include "motion_planning_msgs/PlanSensedApproachAction.h"
#include "motion_msgs/MoveToPostureAction.h"
#include "navigation_msgs/MoveToPositionAction.h"


class PlanSensedApproachAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::PlanSensedApproachAction> as_;
  std::string action_name_;
  motion_planning_msgs::PlanSensedApproachFeedback feedback_;
  motion_planning_msgs::PlanSensedApproachResult result_;

 public:
  PlanSensedApproachAction(ros::NodeHandle nh, std::string name);

  ~PlanSensedApproachAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void timerCB(const ros::TimerEvent& event);


 private:
  // detect action client
  actionlib::SimpleActionClient<sensing_msgs::DetectObjectAction> detect_ac_;
  actionlib::SimpleActionClient<motion_msgs::MoveToPostureAction> ac_move_;
  actionlib::SimpleActionClient<navigation_msgs::MoveToPositionAction> ac_nav_;

  // message goals
  motion_msgs::MoveToPostureGoal posture_goal_;
  navigation_msgs::MoveToPositionGoal position_goal_;

  // goal received
  geometry_msgs::Point aoi_position_;
};

geometry_msgs::Point getRelativePosition(geometry_msgs::Point aoi);

#endif /* PLAN_SENSED_APPROACH_SERVER_HPP */
