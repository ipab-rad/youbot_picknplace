#ifndef PLAN_PICK_SERVER_HPP
#define PLAN_PICK_SERVER_HPP

// System
#include <cmath>
// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
// Messages
#include <motion_planning_msgs/PlanPickAction.h>
#include <motion_msgs/MoveToPoseAction.h>
#include <motion_msgs/MoveToPostureAction.h>
#include <motion_msgs/MoveGripperAction.h>

// tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

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

 private:
  // action client to execute pick movement
  actionlib::SimpleActionClient<motion_msgs::MoveGripperAction> ac_gripper_;
  actionlib::SimpleActionClient<motion_msgs::MoveToPoseAction> ac_move_;
  actionlib::SimpleActionClient<motion_msgs::MoveToPostureAction> ac_move_posture_;

  // gripper server
  ros::ServiceClient pose_c_;


  geometry_msgs::PoseStamped object_pose_;
  motion_msgs::MoveToPoseGoal arm_goal_;
  motion_msgs::MoveGripperGoal gripper_goal_;
  motion_msgs::MoveToPostureGoal arm_posture_goal_;

  double approach_dist_;
  double min_grasp_dist_;

};
geometry_msgs::Quaternion computeGripperGraspPose(geometry_msgs::Point pt);
geometry_msgs::Point computeSuggestedMovement(double min_grasp, geometry_msgs::Point pt);

#endif /* PLAN_PICK_SERVER_HPP */
