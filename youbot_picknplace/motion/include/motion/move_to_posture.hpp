#ifndef MOVE_TO_POSTURE_SERVER_HPP
#define MOVE_TO_POSTURE_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include <motion_msgs/MoveToPostureAction.h>

class MoveToPostureAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_msgs::MoveToPostureAction> as_;
  std::string action_name_;
  motion_msgs::MoveToPostureFeedback feedback_;
  motion_msgs::MoveToPostureResult result_;

 public:
  MoveToPostureAction(ros::NodeHandle nh, std::string name);

  ~MoveToPostureAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void timerCB(const ros::TimerEvent& event);


 private:
  std::string target_posture_;

  // threshold
  double distance_threshold_;

  // timer for action timeout
  ros::Timer timer_;
  bool timed_out_;



};


#endif /* MOVE_TO_POSTURE_SERVER_HPP */
