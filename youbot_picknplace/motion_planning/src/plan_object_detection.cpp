#include "motion_planning/plan_object_detection.hpp"
// #include <actionlib/client/terminal_state.h>

PlanObjectDetectionAction::PlanObjectDetectionAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  detect_ac_("sensing/object_detection", true) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanObjectDetectionAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanObjectDetectionAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting PlanObjectDetection server");
  as_.start();
}

PlanObjectDetectionAction::~PlanObjectDetectionAction(void) {
}

void PlanObjectDetectionAction::init() {

}

void PlanObjectDetectionAction::goalCB() {
  detect_ = as_.acceptNewGoal()->detect;
  this->executeCB();
}

void PlanObjectDetectionAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void PlanObjectDetectionAction::executeCB() {
  bool going = true;
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  feedback_.curr_state = 1;
  as_.publishFeedback(feedback_);

  // send a goal to the obj detection action
  sensing_msgs::DetectObjectGoal detect_goal;
  detect_goal.detect = true;
  detect_ac_.sendGoal(detect_goal);

  // start timer
  timed_out_ = false;
  timer_ = nh_.createTimer(ros::Duration(30), &PlanObjectDetectionAction::timerCB, this, true);

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // debug: monitor action
    // ROS_INFO("Current State: %s\n", detect_ac_.getState().toString().c_str());
    if (detect_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      success = true;
      going = false;
    }

    if (timed_out_) {
      // detect_ac_.cancelGoal();
      ROS_INFO("%s: Timed out", action_name_.c_str());
      going = false;
    }

    ros::spinOnce();
    r.sleep();
  }

  feedback_.curr_state = 2;
  as_.publishFeedback(feedback_);

  if (success) {
    result_.success = success;
    result_.pose = detect_ac_.getResult()->pose;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = success;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }

}

void PlanObjectDetectionAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}
