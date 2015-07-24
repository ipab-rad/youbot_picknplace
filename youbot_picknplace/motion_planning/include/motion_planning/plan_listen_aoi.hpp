#ifndef PLAN_LISTEN_AOI_SERVER_HPP
#define PLAN_LISTEN_AOI_SERVER_HPP

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
// Messages
#include "motion_planning_msgs/PlanListenAoiAction.h"
#include "navigation_msgs/MoveToPositionAction.h"


class PlanListenAoiAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_planning_msgs::PlanListenAoiAction> as_;
  std::string action_name_;
  motion_planning_msgs::PlanListenAoiFeedback feedback_;
  motion_planning_msgs::PlanListenAoiResult result_;

 public:
  PlanListenAoiAction(ros::NodeHandle nh, std::string name);

  ~PlanListenAoiAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void timerCB(const ros::TimerEvent& event);

  void aoiCB(const geometry_msgs::Point::ConstPtr& msg);


 private:
  // area of interest listener
  ros::Subscriber aoi_sub_;
  bool found_;
  int waiting_time_;
  // navigation client
  actionlib::SimpleActionClient<navigation_msgs::MoveToPositionAction> ac_move_;

  

  // message goals
  navigation_msgs::MoveToPositionGoal position_goal_;
  // position goal
  geometry_msgs::Point target_position_;


  // timer for action timeout
  ros::Timer timer_;
  bool timed_out_;

};


#endif /* PLAN_LISTEN_AOI_SERVER_HPP */
