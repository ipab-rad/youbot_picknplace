#ifndef PLAN_OBJECT_DETECTION_SERVER_HPP
#define PLAN_OBJECT_DETECTION_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
// Messages
#include "sensing_msgs/DetectObjectAction.h"
#include "motion_planning_msgs/PlanObjectDetectionAction.h"
#include "motion_msgs/MoveToPostureAction.h"


class PlanObjectDetectionAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::PlanObjectDetectionAction> as_;
  std::string action_name_;
  motion_planning_msgs::PlanObjectDetectionFeedback feedback_;
  motion_planning_msgs::PlanObjectDetectionResult result_;

 public:
  PlanObjectDetectionAction(ros::NodeHandle nh, std::string name);

  ~PlanObjectDetectionAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void timerCB(const ros::TimerEvent& event);


 private:

  bool detect_;
  int initial_state_;
  // detect action client
  actionlib::SimpleActionClient<sensing_msgs::DetectObjectAction> detect_ac_;
  actionlib::SimpleActionClient<motion_msgs::MoveToPostureAction> ac_move_;

  // message goals
  motion_msgs::MoveToPostureGoal posture_goal_;


  // timer for action timeout
  ros::Timer timer_;
  bool timed_out_;

};


#endif /* PLAN_OBJECT_DETECTION_SERVER_HPP */
