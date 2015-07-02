#ifndef PLAN_PICK_SERVER_HPP
#define PLAN_PICK_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// #include <actionlib/client/simple_action_client.h>
// Messages
#include <motion_planning_msgs/PlanPickAction.h>
// #include <motion_msgs/MoveToPoseAction.h>


class PlanPickAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::PlanPickAction> as_;
  std::string action_name_;
  motion_planning_msgs::PlanPickFeedback feedback_;
  motion_planning_msgs::PlanPickResult result_;

 public:
  PlanPickAction(ros::NodeHandle nh, std::string name);

  ~PlanPickAction(void);

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


#endif /* PLAN_PICK_SERVER_HPP */
