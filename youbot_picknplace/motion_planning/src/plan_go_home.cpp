#include "motion_planning/plan_go_home.hpp"

PlanGoHomeAction::PlanGoHomeAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  ac_move_("motion/move_to_posture", true) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanGoHomeAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanGoHomeAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting PlanGoHome server");
  as_.start();
}

PlanGoHomeAction::~PlanGoHomeAction(void) {
}

void PlanGoHomeAction::init() {
}

void PlanGoHomeAction::goalCB() {
  go_home_ = as_.acceptNewGoal()->go_home;
  this->executeCB();
}

void PlanGoHomeAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void PlanGoHomeAction::executeCB() {
  bool going = true;
  bool success = false;

  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  posture_goal_.posture = "folded";
  // move to pose action
  // ac_move_.waitForServer();
  ROS_INFO("Moving to home position (initial)");
  ac_move_.sendGoal(posture_goal_);

  // arm moving
  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      success = true;
      going = false;
    } else if (ac_move_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      success = false;
      going = false;
    }

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
