#include "motion/move_to_position.hpp"
#include "motion_msgs/MoveToPositionAction.h"


MoveToPositionAction::MoveToPositionAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&MoveToPositionAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&MoveToPositionAction::preemptCB, this));

  // TODO:
  // will need subscriber to monitor joint positions, possible feeback
  joint_pos_sub_ = nh_.subscribe("/joint_states", 1,
                                &MoveToPositionAction::positionCB, this);
  // and service client for planning motion
  planning_client_ = nh_.serviceClient<motion_planning_msgs::PlanMotion>(
                          "/motion_planning/plan_motion", true);
  planning_client_.waitForExistence();

  this->init();
  ROS_INFO("Starting MoveToPosition server");
  as_.start();
}

MoveToPositionAction::~MoveToPositionAction(void) {
}

void MoveToPositionAction::init() {
  // OLD CODE:
  // ball_found_ = false;
  // target_distance_ = 0.0f;
  // target_theta_ = 0.0f;
  // dist_scalar_ = 3.0f;
  // theta_scalar_ = 1.0f;
  // dist_thresh_ = 0.05f;
  // theta_thresh_ = 0.2f;
  // ball_lost_monitor_srv_.request.monitor_mode = MonitorMode::BALL_LOST;
}

void MoveToPositionAction::positionCB(const sensor_msgs::JointState::ConstPtr& msg) {
  joint_pos_ = msg->position;
}

void MoveToPositionAction::goalCB() {
  target_position_ = as_.acceptNewGoal()->position;
  this->executeCB();
}

void MoveToPositionAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void MoveToPositionAction::executeCB() {
  bool going = true;
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // Set planning request
    planning_srv_.request.x = target_position_.x;
    planning_srv_.request.x = target_position_.y;
    planning_srv_.request.x = target_position_.z;
    

    // calling service
    if (planning_client_.call(planning_srv_)) {
      ROS_INFO("Reached planning service %s", action_name_.c_str());
      success = true;
      going = false;
    }else{
      ROS_INFO("Planning service %s was not reached", action_name_.c_str());
      success = false;
      going = false;
    }
    ros::spinOnce();
    r.sleep();
  }
 
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