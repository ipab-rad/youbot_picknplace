#ifndef PLAN_APPROACH_SERVER_HPP
#define PLAN_APPROACH_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// #include <actionlib/client/simple_action_client.h>
// Messages
#include <motion_planning_msgs/PlanApproachAction.h>
// #include <motion_msgs/MoveToPoseAction.h>


class PlanApproachAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::PlanApproachAction> as_;
  std::string action_name_;
  motion_planning_msgs::PlanApproachFeedback feedback_;
  motion_planning_msgs::PlanApproachResult result_;

 public:
  PlanApproachAction(ros::NodeHandle nh, std::string name);

  ~PlanApproachAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void timerCB(const ros::TimerEvent& event);


 private:
  // action client to execute approach movement
  // actionlib::SimpleActionClient<motion_msgs::MoveToPoseAction> ac_;



  geometry_msgs::PoseStamped object_pose_;

  // timer for action timeout
  ros::Timer timer_;
  bool timed_out_;



};


#endif /* PLAN_APPROACH_SERVER_HPP */
