#ifndef PLAN_APPROACH_OBJECT_SERVER_HPP
#define PLAN_APPROACH_OBJECT_SERVER_HPP

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
// Messages
#include "motion_planning_msgs/PlanApproachObjectAction.h"
#include "navigation_msgs/MoveToPositionAction.h"


class PlanApproachObjectAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::PlanApproachObjectAction> as_;
  std::string action_name_;
  motion_planning_msgs::PlanApproachObjectFeedback feedback_;
  motion_planning_msgs::PlanApproachObjectResult result_;

 public:
  PlanApproachObjectAction(ros::NodeHandle nh, std::string name);

  ~PlanApproachObjectAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

 private:
  // navigation client
  actionlib::SimpleActionClient<navigation_msgs::MoveToPositionAction> ac_move_;

  // message goals
  navigation_msgs::MoveToPositionGoal position_goal_;
  // position goal
  geometry_msgs::Point target_position_;

};


#endif /* PLAN_APPROACH_OBJECT_SERVER_HPP */
