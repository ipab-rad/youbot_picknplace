#include "motion_planning/plan_place.hpp"

PlanPlaceAction::PlanPlaceAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  ac_gripper_("gripper_motion/move_gripper", true),
  ac_move_("motion/move_to_posture", true) {
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
  // states
  // 0 initial
  // 1 moved
  // 2 placed
  int state = 0;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if ( state == 0) {
      // send a movement to pose goal to the action
      posture_goal_.posture = "back_drop";
      // move to pose action
      ac_move_.waitForServer();
      ROS_INFO("Placing down object");

      ac_move_.sendGoal(posture_goal_);
      feedback_.curr_state = 1;
      as_.publishFeedback(feedback_);
      ac_move_.waitForResult();
      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 1;
      }
    } else if (state == 1) {
      // open gripper action
      gripper_goal_.command = 1;
      ac_gripper_.waitForServer();
      ROS_INFO("Opening Gripper");
      ac_gripper_.sendGoal(gripper_goal_);
      ac_gripper_.waitForResult();
      if (ac_gripper_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        success = true;
      } else {
        success = false;
      }
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

void PlanPlaceAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}
