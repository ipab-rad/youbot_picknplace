#include "motion_planning/plan_approach_object.hpp"

PlanApproachObjectAction::PlanApproachObjectAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  ac_move_("navigation/move_to_position", true) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanApproachObjectAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanApproachObjectAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting PlanApproachObject server");
  as_.start();
}

PlanApproachObjectAction::~PlanApproachObjectAction(void) {
}

void PlanApproachObjectAction::init() {
}

void PlanApproachObjectAction::goalCB() {
  target_position_ = as_.acceptNewGoal()->position;
  this->executeCB();
}

void PlanApproachObjectAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void PlanApproachObjectAction::executeCB() {
  bool success = false;
  bool going = true;

  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  // move to pose action
  ac_move_.waitForServer();
  position_goal_.position = target_position_;
  position_goal_.relative = true;
  ROS_INFO("Navigating to object");
  ac_move_.sendGoal(position_goal_);

  // base moving
  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }


    if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      // success
      success = true;
      going = false;
    } else if (ac_move_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      // failed
      success = false;
      going = false;
    }

    ros::spinOnce();
    r.sleep();
  }

  if (success) {
    result_.success = success;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = success;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }

}
