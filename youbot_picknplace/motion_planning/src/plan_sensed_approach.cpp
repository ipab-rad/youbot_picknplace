#include "motion_planning/plan_sensed_approach.hpp"

PlanSensedApproachAction::PlanSensedApproachAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  detect_ac_("sensing/object_detection", true),
  ac_move_("motion/move_to_posture", true),
  ac_nav_("navigation/move_to_position", true) {

  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanSensedApproachAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanSensedApproachAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting PlanSensedApproach server");
  as_.start();
}

PlanSensedApproachAction::~PlanSensedApproachAction(void) {
}

void PlanSensedApproachAction::init() {
}

void PlanSensedApproachAction::goalCB() {
  aoi_position_ = as_.acceptNewGoal()->aoi_position;
  this->executeCB();
}

void PlanSensedApproachAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void PlanSensedApproachAction::executeCB() {
  bool going = true;
  bool success = false;
  // moving states
  bool moving_arm = false;
  bool moving_base = false;
  // detecting state
  bool detecting = false;

  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  //  states:
  // 0 arm positioning
  // 1 check front pt1
  // 2 check front pt2
  // 3 check front pt3
  // 4 end
  int state = 0;
  int endstate = 4;

  // declare some set messages
  // send a goal to the obj detection action
  sensing_msgs::DetectObjectGoal detect_goal;
  detect_goal.detect = true;
  detect_goal.timeout = 10;

  // move to initial detection position
  position_goal_.position = getRelativePosition(aoi_position_);
  position_goal_.relative = true;

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // performing detection
    if (detecting) {
      if (detect_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        detecting = false;
        success = true;
        going = false;
      } else if (detect_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        detecting = false;
        state++;
        if (state == endstate) {
          going = false;
        }
      }
    } else if (moving_arm) {
      // arm is moving
      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        detect_ac_.sendGoal(detect_goal);
        detecting = true;
        moving_arm = false;
        // start timer
      } else if (ac_move_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        going = false;
        success = false;
        moving_arm = false;
      }
    } else if (moving_base) {
      // navigating to new position
      if (ac_nav_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        // send a goal to the obj detection action
        detect_ac_.sendGoal(detect_goal);
        detecting = true;
        moving_base = false;
        // start timer
      } else if (ac_nav_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        going = false;
        success = false;
        moving_base = false;
      }
    } else if (state == 0) {
      // move to initial detection position
      posture_goal_.posture = "sensed_approach_front";
      ROS_INFO("Checking for object with posture %s", posture_goal_.posture.c_str());
      ac_move_.sendGoal(posture_goal_);
      moving_arm = true;
    } else if (state < endstate) {
      ROS_INFO("Approaching 20cm");
      ac_nav_.sendGoal(position_goal_);
      moving_base = true;
    }

    ros::spinOnce();
    r.sleep();
  }

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

// compute desired position (20cm away) to move manipulating agent to given an AOI
geometry_msgs::Point getRelativePosition(geometry_msgs::Point aoi) {
  double magnitude = 0.2;
  geometry_msgs::Point result;
  double direction = atan2(aoi.y, aoi.x);
  result.x = magnitude * cos(direction);
  result.y = magnitude * sin(direction);
  result.z = 0.0;
  return result;
}
