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

  void goalCB();

  void preemptCB();

  void executeCB();

  void timerCB(const ros::TimerEvent& event);


 private:
  geometry_msgs::Point target_position_;

  // threshold
  double distance_threshold_;

  // timer for action timeout
  ros::Timer timer_;
  bool timed_out_;



};


#endif /* MOVE_TO_POSITION_SERVER_HPP */
