#ifndef MOVE_GRIPPER_SERVER_HPP
#define MOVE_GRIPPER_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
// Messages
#include <motion_msgs/MoveGripperAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


class MoveGripperAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<motion_msgs::MoveGripperAction> as_;
  std::string action_name_;
  motion_msgs::MoveGripperFeedback feedback_;
  motion_msgs::MoveGripperResult result_;

 public:
  MoveGripperAction(ros::NodeHandle nh, std::string name);

  ~MoveGripperAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void timerCB(const ros::TimerEvent& event);


 private:
  int gripper_command_;
  trajectory_msgs::JointTrajectory trajectory_;
  trajectory_msgs::JointTrajectoryPoint trajectory_pt1_;
  trajectory_msgs::JointTrajectoryPoint trajectory_pt2_;
  std::string joint_name1_;
  std::string joint_name2_;

  std::string gripper_topic_;
  ros::Publisher gripper_pub_;
  // timer for action timeout
  ros::Timer timer_;
  bool timed_out_;



};


#endif /* MOVE_GRIPPER_SERVER_HPP */
