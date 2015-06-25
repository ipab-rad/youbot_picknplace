#ifndef MOVE_TO_POSITION_SERVER_HPP
#define MOVE_TO_POSITION_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include <sensor_msgs/JointState.h>
#include <youbot_picknplace/MoveToPositionAction.h>
#include "youbot_picknplace/PlanMotion.h"

class MoveToPositionAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<youbot_picknplace::MoveToPositionAction> as_;
  std::string action_name_;
  youbot_picknplace::MoveToPositionFeedback feedback_;
  youbot_picknplace::MoveToPositionResult result_;

 public:
  MoveToPositionAction(ros::NodeHandle nh, std::string name);

  ~MoveToPositionAction(void);

  void init();

  void positionCB(const sensor_msgs::JointState::ConstPtr& msg);

  void goalCB();

  void preemptCB();

  void executeCB();

 private:
  geometry_msgs::Point target_position_;

  ros::Subscriber joint_pos_sub_;
  std::vector<double> joint_pos_;

  ros::ServiceClient planning_client_;
  youbot_picknplace::PlanMotion planning_srv_;
};


#endif /* MOVE_TO_POSITION_SERVER_HPP */
