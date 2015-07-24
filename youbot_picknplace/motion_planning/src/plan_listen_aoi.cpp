#include "motion_planning/plan_listen_aoi.hpp"

PlanListenAoiAction::PlanListenAoiAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  aoi_sub_(nh.subscribe("/areas_of_interest", 1000, &PlanListenAoiAction::aoiCB,this)),
  ac_move_("navigation/move_to_position", true) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanListenAoiAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanListenAoiAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting PlanListenAoi server");
  as_.start();
}

PlanListenAoiAction::~PlanListenAoiAction(void) {
}

void PlanListenAoiAction::init() {
  waiting_time_ = 30;
}

void PlanListenAoiAction::goalCB() {
  as_.acceptNewGoal();
  this->executeCB();
}

void PlanListenAoiAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void PlanListenAoiAction::executeCB() {
  bool going = true;
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  timed_out_ = false;
  timer_ = nh_.createTimer(ros::Duration(waiting_time_), &PlanListenAoiAction::timerCB, this, true);
  
  found_ = false;

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if (found_) {
      // move to pose action
      ac_move_.waitForServer();
      position_goal_.position = target_position_;
      position_goal_.relative = true;
      ROS_INFO("Navigating to area of interest");
      ac_move_.sendGoal(position_goal_);
      ac_move_.waitForResult();

      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        // success
        success = true;
      }
      going = false;
    }

    if (timed_out_) {
      ROS_INFO("%s: Timed out", action_name_.c_str());
      // terminate
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

void PlanListenAoiAction::timerCB(const ros::TimerEvent & event) {
  timed_out_ = true;
}

void PlanListenAoiAction::aoiCB(const geometry_msgs::Point::ConstPtr& msg){
  if(!found_){
    target_position_.x = msg->x;
    target_position_.y = msg->y;
    target_position_.z = msg->z;
    found_ = true;
  }
}