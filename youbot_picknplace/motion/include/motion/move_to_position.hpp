#ifndef MOVE_TO_POSITION_SERVER_HPP
#define MOVE_TO_POSITION_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include <sensor_msgs/JointState.h>
#include <motion_msgs/MoveToPositionAction.h>
#include <motion_planning_msgs/PlanMotion.h>

class MoveToPositionAction {
 protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_msgs::MoveToPositionAction> as_;
  std::string action_name_;
  motion_msgs::MoveToPositionFeedback feedback_;
  motion_msgs::MoveToPositionResult result_;

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
  motion_planning_msgs::PlanMotion planning_srv_;
};


#endif /* MOVE_TO_POSITION_SERVER_HPP */
