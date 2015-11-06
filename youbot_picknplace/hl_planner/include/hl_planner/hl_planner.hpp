/**
 * @file      hl_planner.hpp
 * @brief     High-Level Planner executes a list of action instructions
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-11-03
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#ifndef HL_PLANNER_HPP
#define HL_PLANNER_HPP

#include <ros/ros.h>
#include <csignal>
#include <fstream>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <motion_msgs/MoveToPostureAction.h>
#include <motion_msgs/MoveGripperAction.h>
#include <motion_planning_msgs/PlanPickAction.h>
#include <sensing_msgs/DetectObjectAction.h>

class HLPlanner {
 public:
  HLPlanner(ros::NodeHandle* nh);
  ~HLPlanner();

  void init();
  void rosSetup();
  void loadParams();

  void execute();
  bool isFinished() {return finished_;}
  static void interrupt(int s);
  static bool isInterrupted() {return interrupted_;}

 private:

  void pick_a();
  void pick_b();
  void goto_a();
  void goto_b();
  void place_a();
  void place_b();
  void drop_a();
  void drop_b();

  void initRobot();
  void endRobot();

  // Flags
  bool finished_;
  bool failed_;
  static bool interrupted_;

  // Constants
  std::string PICK_A;
  std::string PICK_B;
  std::string GOTO_A;
  std::string GOTO_B;
  std::string PLACE_A;
  std::string PLACE_B;
  std::string plan_path_;
  std::string plan_file_;

  ros::Duration wait_init_;
  ros::Duration wait_;
  ros::Duration wait_detect_;

  // Variables
  std::string robot_state_; // A or B
  std::string cube_a_state_; // A, B or R
  std::string cube_b_state_; // A, B or R
  std::vector<std::string> action_list_;

  geometry_msgs::PoseStamped cube_a_pose_;
  geometry_msgs::PoseStamped cube_b_pose_;

  // ROS
  ros::NodeHandle* nh_;
  actionlib::SimpleActionClient<motion_msgs::MoveToPostureAction> posture_ac_;
  actionlib::SimpleActionClient<motion_msgs::MoveGripperAction> gripper_ac_;
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPickAction> pick_ac_;
  actionlib::SimpleActionClient<sensing_msgs::DetectObjectAction> detect_ac_;
  motion_msgs::MoveGripperGoal close_;
  motion_msgs::MoveGripperGoal open_;
  motion_msgs::MoveToPostureGoal candle_;
  motion_msgs::MoveToPostureGoal place_;
  motion_msgs::MoveToPostureGoal drop_a_;
  motion_msgs::MoveToPostureGoal drop_b_;
  motion_msgs::MoveToPostureGoal home_;
  motion_planning_msgs::PlanPickGoal pick_;
  sensing_msgs::DetectObjectGoal detect_;
};

#endif /* HL_PLANNER_HPP */
