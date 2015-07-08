#ifndef PLAN_GO_HOME_SERVER_HPP
#define PLAN_GO_HOME_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// Messages
#include "motion_planning_msgs/PlanGoHomeAction.h"
#include "motion_msgs/MoveToPostureAction.h"


class PlanGoHomeAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::PlanGoHomeAction> as_;
  std::string action_name_;
  motion_planning_msgs::PlanGoHomeFeedback feedback_;
  motion_planning_msgs::PlanGoHomeResult result_;

 public:
  PlanGoHomeAction(ros::NodeHandle nh, std::string name);

  ~PlanGoHomeAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

 private:
  // action client to execute go home movement
  actionlib::SimpleActionClient<motion_msgs::MoveToPostureAction> ac_move_;

  // goal
  bool go_home_;
  // message goals
  motion_msgs::MoveToPostureGoal posture_goal_;

};


#endif /* PLAN_GO_HOME_SERVER_HPP */
