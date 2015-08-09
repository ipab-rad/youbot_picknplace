#include "motion_planning/plan_listen_aoi.hpp"

PlanListenAoiAction::PlanListenAoiAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  aoi_sub_(nh.subscribe("/areas_of_interest", 1000, &PlanListenAoiAction::aoiCB, this)),
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

  // 1 base moving to object
  // 0 listening for area of interest
  int state = 0;
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  found_ = false;

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if (found_) {
      if (state == 0) {
        // move to position action
        ac_move_.waitForServer();
        position_goal_.position = getApproachablePosition(target_position_);
        position_goal_.relative = true;
        ROS_INFO("Approaching Area of Interest. Going to (%f,%f) relative to current position.",
                 position_goal_.position.x, position_goal_.position.y);
        ac_move_.sendGoal(position_goal_);
        state = 1;

      } else if (state == 1) {
        // base moving
        if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
          // success
          success = true;
          going = false;
        } else if (ac_move_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
          success = false;
          going = false;
        }
      }
    }

    ros::spinOnce();
    r.sleep();
  }

  if (success) {
    result_.success = success;
    result_.aoi_position = target_position_;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = success;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }

}

void PlanListenAoiAction::aoiCB(const geometry_msgs::Point::ConstPtr& msg) {
  if (!found_) {
    target_position_.x = msg->x;
    target_position_.y = msg->y;
    target_position_.z = msg->z;
    found_ = true;
  }
}

// compute desired position to move manipulating agent to given an AOI
geometry_msgs::Point getApproachablePosition(geometry_msgs::Point aoi) {
  double magnitude = sqrt(pow(aoi.x, 2) + pow(aoi.y, 2));
  double safety_dist = 1.0; // 1m away from AOI
  geometry_msgs::Point result;
  if (magnitude > safety_dist) {
    double direction = atan2(aoi.y, aoi.x);
    result.x = (magnitude - safety_dist) * cos(direction);
    result.y = (magnitude - safety_dist) * sin(direction);
    result.z = 0.0;
  } else {
    ROS_FATAL("AOI is less than 1m away");
  }
  return result;
}
