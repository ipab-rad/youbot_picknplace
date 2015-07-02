#include "motion_planning/plan_approach.hpp"
#include "motion_planning_msgs/PlanApproachAction.h"
#include "motion_msgs/MoveToPoseAction.h"
#include <actionlib/client/simple_action_client.h>
// #include <actionlib/client/terminal_state.h>

 //tf
#include <tf/transform_datatypes.h>

PlanApproachAction::PlanApproachAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name)
  {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanApproachAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanApproachAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting PlanApproach server");
  as_.start();
}

PlanApproachAction::~PlanApproachAction(void) {
}

void PlanApproachAction::init() {
}

void PlanApproachAction::goalCB() {
  object_pose_ = as_.acceptNewGoal()->object_pose;
  this->executeCB();
}

void PlanApproachAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void PlanApproachAction::executeCB() {
  bool going = true;
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  // send a movement to pose goal to the action
  motion_msgs::MoveToPoseGoal goal;
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_footprint";
  geometry_msgs::Quaternion quat;
  quat = tf::createQuaternionMsgFromRollPitchYaw(-3.129,0.0549,1.686);
  target_pose.pose.orientation.x = quat.x;
  target_pose.pose.orientation.y = quat.y;
  target_pose.pose.orientation.z = quat.z;
  target_pose.pose.orientation.w = quat.w;
  ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
  target_pose.pose.position.x = object_pose_.pose.position.x;
  target_pose.pose.position.y = object_pose_.pose.position.y;
  target_pose.pose.position.z = 0.1;
  goal.pose = target_pose;
    // move to pose action
  actionlib::SimpleActionClient<motion_msgs::MoveToPoseAction> ac_("motion/move_to_pose", true);
  ac_.waitForServer();
  ROS_INFO("Server has connected");

  ac_.sendGoal(goal);

  going = false;
  success = true;

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // TODO
    // print status of the action running

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

void PlanApproachAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}
