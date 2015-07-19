#ifndef MOVE_TO_POSITION_SERVER_HPP
#define MOVE_TO_POSITION_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include "navigation_msgs/MoveToPositionAction.h"
#include "nav_msgs/Odometry.h"

class MoveToPositionAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<navigation_msgs::MoveToPositionAction> as_;
  std::string action_name_;
  navigation_msgs::MoveToPositionFeedback feedback_;
  navigation_msgs::MoveToPositionResult result_;

 public:
  MoveToPositionAction(ros::NodeHandle nh, std::string name);

  ~MoveToPositionAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void timerCB(const ros::TimerEvent& event);

  void setTargetTwist(double x, double y, bool stop);

  void currPositionCallback(const nav_msgs::Odometry::ConstPtr& msg);


 private:
  geometry_msgs::Point target_position_;
  geometry_msgs::Point curr_position_;
  bool odom_received_;

  // threshold
  double distance_tol_;
  double distance_;

  // moving velocity
  double std_vel_;

  // timer for action timeout
  ros::Timer timer_;
  bool timed_out_;

  // navigation pub and sub
  ros::Subscriber nav_sub_;
  ros::Publisher nav_pub_;
  geometry_msgs::Twist nav_pub_msg_;


};


#endif /* MOVE_TO_POSITION_SERVER_HPP */
