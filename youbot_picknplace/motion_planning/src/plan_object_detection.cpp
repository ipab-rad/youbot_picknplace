#include "motion_planning/plan_object_detection.hpp"

PlanObjectDetectionAction::PlanObjectDetectionAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  detect_ac_("sensing/object_detection", true),
  ac_move_("motion/move_to_posture", true) {
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
  detect_ = false;
}

void PlanObjectDetectionAction::goalCB() {
  as_.acceptNewGoal();
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

  int detection_time = 30;


  //  states:
  // 0 check front
  // 1 check right
  // 2 check left
  int state = 0;

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if (!detect_) {
      if (state == 0)
        posture_goal_.posture = "check_front";
      else if (state == 1)
        posture_goal_.posture = "check_right";
      else if (state == 2)
        posture_goal_.posture = "check_left";

      // move to pose action
      ac_move_.waitForServer();
      ROS_INFO("Checking for object with posture %s", posture_goal_.posture.c_str());
      ac_move_.sendGoal(posture_goal_);
      feedback_.curr_state = 1;
      as_.publishFeedback(feedback_);
      ac_move_.waitForResult();
      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        // send a goal to the obj detection action
        sensing_msgs::DetectObjectGoal detect_goal;
        detect_goal.detect = true;
        detect_ac_.sendGoal(detect_goal);
        detect_ = true;
        // start timer
        timed_out_ = false;
        timer_ = nh_.createTimer(ros::Duration(detection_time), &PlanObjectDetectionAction::timerCB, this, true);
      }
    }


    // debug: monitor action
    // ROS_INFO("Current State: %s\n", detect_ac_.getState().toString().c_str());
    if (detect_) {
      if (detect_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        detect_ = false;
        success = true;
        going = false;
      }
    }

    if (timed_out_) {
      detect_ = false;
      // send a goal to the obj detection action
      sensing_msgs::DetectObjectGoal detect_goal;
      detect_goal.detect = false;
      detect_ac_.sendGoal(detect_goal);

      if (state == 2) {
        ROS_INFO("%s: Timed out", action_name_.c_str());
        // terminate
        going = false;

      } else
        state++;
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

void PlanObjectDetectionAction::timerCB(const ros::TimerEvent & event) {
  timed_out_ = true;
}
