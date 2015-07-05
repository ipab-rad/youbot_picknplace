#ifndef MOVE_TO_POSE_SERVER_HPP
#define MOVE_TO_POSE_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include "motion_msgs/MoveToPoseAction.h"
// MoveIt
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class MoveToPoseAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_msgs::MoveToPoseAction> as_;
  std::string action_name_;
  motion_msgs::MoveToPoseFeedback feedback_;
  motion_msgs::MoveToPoseResult result_;

 public:
  MoveToPoseAction(ros::NodeHandle nh, std::string name);

  ~MoveToPoseAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void timerCB(const ros::TimerEvent& event);


 private:
  geometry_msgs::PoseStamped target_pose_;
  geometry_msgs::PoseStamped curr_pose_;

  // threshold
  double distance_threshold_;
  double distance_;

  // timer for action timeout
  ros::Timer timer_;
  bool timed_out_;

  // moveit motion plan
  move_group_interface::MoveGroup::Plan plan;



};


#endif /* MOVE_TO_POSE_SERVER_HPP */
