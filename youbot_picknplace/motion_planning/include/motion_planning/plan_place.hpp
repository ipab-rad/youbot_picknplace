#ifndef PLAN_PLACE_SERVER_HPP
#define PLAN_PLACE_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// Messages
#include "motion_planning_msgs/PlanPlaceAction.h"
#include "motion_msgs/MoveToPostureAction.h"
#include "motion_msgs/MoveGripperAction.h"


class PlanPlaceAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::PlanPlaceAction> as_;
  std::string action_name_;
  motion_planning_msgs::PlanPlaceFeedback feedback_;
  motion_planning_msgs::PlanPlaceResult result_;

 public:
  PlanPlaceAction(ros::NodeHandle nh, std::string name);

  ~PlanPlaceAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void timerCB(const ros::TimerEvent& event);


 private:
  // action client to execute approach movement
  // actionlib::SimpleActionClient<motion_msgs::MoveToPoseAction> ac_;

  bool place_object_;

  // timer for action timeout
  ros::Timer timer_;
  bool timed_out_;



};


#endif /* PLAN_PLACE_SERVER_HPP */
