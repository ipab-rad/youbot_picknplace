#include "motion_planning/plan_pick.hpp"
#include "motion_planning_msgs/PlanPickAction.h"
#include "motion_msgs/MoveToPoseAction.h"
#include "motion_msgs/MoveGripperAction.h"
#include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>

//tf
#include <tf/transform_datatypes.h>

PlanPickAction::PlanPickAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanPickAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanPickAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting PlanPick server");
  as_.start();
}

PlanPickAction::~PlanPickAction(void) {
}

void PlanPickAction::init() {
}

void PlanPickAction::goalCB() {
  object_pose_ = as_.acceptNewGoal()->object_pose;
  this->executeCB();
}

void PlanPickAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void PlanPickAction::executeCB() {
  bool going = true;
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  // open gripper action
  actionlib::SimpleActionClient<motion_msgs::MoveGripperAction> ac_gripper_("gripper_motion/move_gripper", true);
  motion_msgs::MoveGripperGoal open_gripper_goal;
  open_gripper_goal.command = 1;
  ac_gripper_.waitForServer();
  ROS_INFO("Opening Gripper");
  ac_gripper_.sendGoal(open_gripper_goal);

  feedback_.curr_state = 1;
  as_.publishFeedback(feedback_);

  sleep(5.0);

  // send a movement to pose goal to the action
  actionlib::SimpleActionClient<motion_msgs::MoveToPoseAction> ac_move_("motion/move_to_pose", true);
  motion_msgs::MoveToPoseGoal goal;
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_footprint";
  geometry_msgs::Quaternion quat;
  quat = tf::createQuaternionMsgFromRollPitchYaw(-3.129, 0.0549, 1.686);
  target_pose.pose.orientation.x = quat.x;
  target_pose.pose.orientation.y = quat.y;
  target_pose.pose.orientation.z = quat.z;
  target_pose.pose.orientation.w = quat.w;
  ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
  target_pose.pose.position.x = object_pose_.pose.position.x;
  target_pose.pose.position.y = object_pose_.pose.position.y;
  target_pose.pose.position.z = object_pose_.pose.position.z;
  goal.pose = target_pose;
  // move to pose action
  ac_move_.waitForServer();
  ROS_INFO("Picking up object");

  ac_move_.sendGoal(goal);

  sleep(5.0);


  //  close gripper
  open_gripper_goal.command = 0;
  ROS_INFO("Closing Gripper");
  ac_gripper_.sendGoal(open_gripper_goal);


  going = false;
  success = true;

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // TODO
    // print status of the action running

    ros::spinOnce();
    r.sleep();
  }

  feedback_.curr_state = 2;
  as_.publishFeedback(feedback_);

  if (success) {
    result_.success = true;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = false;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }
}

void PlanPickAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}
