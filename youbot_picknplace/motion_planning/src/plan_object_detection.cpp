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
  initial_state_ = 0;
}

void PlanObjectDetectionAction::goalCB() {
  initial_state_ = as_.acceptNewGoal()->detect;
  this->executeCB();
}

void PlanObjectDetectionAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void PlanObjectDetectionAction::executeCB() {
  bool going = true;
  bool success = false;
  // moving state
  bool moving = false;

  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  //  states:
  // 0 check front
  // 1 check right
  // 2 check left
  // 3 end
  int state = initial_state_;
  int endstate = 3;


  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // performing detection
    if (detect_) {
      if (detect_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        detect_ = false;
        success = true;
        going = false;
      } else if (detect_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        detect_ = false;
        state++;
        if (state == endstate) {
          going = false;
        }
      }
    }
    // moving to detection position
    else if (moving) {
      // arm is moving
      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        // send a goal to the obj detection action
        sensing_msgs::DetectObjectGoal detect_goal;
        detect_goal.detect = true;
        detect_goal.timeout = 5;
        detect_ac_.sendGoal(detect_goal);
        detect_ = true;
        moving = false;
        // start timer
      } else if (ac_move_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        going = false;
        success = false;
        moving = false;
      }
    } else if (state == 0) {
      // move to initial detection position
      posture_goal_.posture = "check_front";
      ROS_INFO("Checking for object with posture %s", posture_goal_.posture.c_str());
      ac_move_.sendGoal(posture_goal_);
      moving = true;
    } else if (state == 1) {
      // check right detection position
      posture_goal_.posture = "check_right";
      ROS_INFO("Checking for object with posture %s", posture_goal_.posture.c_str());
      ac_move_.sendGoal(posture_goal_);
      moving = true;
    } else if (state == 2) {
      // check left detection position
      posture_goal_.posture = "check_left";
      ROS_INFO("Checking for object with posture %s", posture_goal_.posture.c_str());
      ac_move_.sendGoal(posture_goal_);
      moving = true;
    }

    ros::spinOnce();
    r.sleep();
  }

  if (success) {
    result_.success = success;
    result_.pose = detect_ac_.getResult()->pose;
    result_.state = state;
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
