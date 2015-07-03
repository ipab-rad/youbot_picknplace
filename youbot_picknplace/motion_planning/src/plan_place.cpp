#include "motion_planning/plan_place.hpp"
#include "motion_planning_msgs/PlanPlaceAction.h"
#include "motion_msgs/MoveToPostureAction.h"
#include "motion_msgs/MoveGripperAction.h"
#include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>

//tf
#include <tf/transform_datatypes.h>

PlanPlaceAction::PlanPlaceAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanPlaceAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanPlaceAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting PlanPlace server");
  as_.start();
}

PlanPlaceAction::~PlanPlaceAction(void) {
}

void PlanPlaceAction::init() {
}

void PlanPlaceAction::goalCB() {
  place_object_ = as_.acceptNewGoal()->place_object;
  this->executeCB();
}

void PlanPlaceAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void PlanPlaceAction::executeCB() {
  bool going = true;
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  // send a movement to pose goal to the action
  actionlib::SimpleActionClient<motion_msgs::MoveToPostureAction> ac_move_("motion/move_to_posture", true);
  motion_msgs::MoveToPostureGoal goal;
  goal.posture = "back_drop";
  // move to pose action
  ac_move_.waitForServer();
  ROS_INFO("Placing down object");

  ac_move_.sendGoal(goal);

  feedback_.curr_state = 1;
  as_.publishFeedback(feedback_);

  sleep(5.0);

  // open gripper action
  actionlib::SimpleActionClient<motion_msgs::MoveGripperAction> ac_gripper_("gripper_motion/move_gripper", true);
  motion_msgs::MoveGripperGoal open_gripper_goal;
  open_gripper_goal.command = 1;
  ac_gripper_.waitForServer();
  ROS_INFO("Opening Gripper");
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

void PlanPlaceAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}
