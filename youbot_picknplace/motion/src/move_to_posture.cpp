#include "motion/move_to_posture.hpp"
#include "motion_msgs/MoveToPostureAction.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

MoveToPostureAction::MoveToPostureAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&MoveToPostureAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&MoveToPostureAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting MoveToPosture server");
  as_.start();
}

MoveToPostureAction::~MoveToPostureAction(void) {
}

void MoveToPostureAction::init() {
  distance_threshold_ = 0.03;
}

void MoveToPostureAction::goalCB() {
  target_posture_ = as_.acceptNewGoal()->posture;
  this->executeCB();
}

void MoveToPostureAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void MoveToPostureAction::executeCB() {
  bool going = true;
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);
  // get move it to execute motion
  moveit::planning_interface::MoveGroup group("arm_1");

  if (going){

    group.setNamedTarget(target_posture_);
    group.setPlanningTime(20.0);
    move_group_interface::MoveGroup::Plan plan;
    if (!group.plan(plan))
    {
      ROS_FATAL("Unable to create motion plan.  Aborting.");
      success = false;
      going = false;
    }else{
      ROS_INFO("Planning was successful");
      // Publish Feedback that plan was success
      feedback_.curr_state = 1;
      as_.publishFeedback(feedback_);

      timed_out_ = false;
      timer_ = nh_.createTimer(ros::Duration(60), &MoveToPostureAction::timerCB, this, true);

      // do non-blocking move request
      group.execute(plan);
      // publish feedback that it is executing motion
      feedback_.curr_state = 2;
      as_.publishFeedback(feedback_);
    }
  }

  
  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if (timed_out_){
      ROS_INFO("%s: Timed out", action_name_.c_str());
      // TODO: set as preempted?
      going = false;
    }

    ros::spinOnce();
    r.sleep();
  }

  feedback_.curr_state = 3;
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

void MoveToPostureAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}