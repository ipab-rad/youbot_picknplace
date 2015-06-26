#include "motion/move_to_position.hpp"
#include "motion_msgs/MoveToPositionAction.h"

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

 //tf
#include <tf/transform_datatypes.h>

MoveToPositionAction::MoveToPositionAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&MoveToPositionAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&MoveToPositionAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting MoveToPosition server");
  as_.start();
}

MoveToPositionAction::~MoveToPositionAction(void) {
}

void MoveToPositionAction::init() {
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
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);
  // get move it to execute motion
  moveit::planning_interface::MoveGroup group("arm_1");

  if (going){
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "base_footprint";
    geometry_msgs::Quaternion quat ;
    quat = tf::createQuaternionMsgFromRollPitchYaw(-3.129,0.0549,1.686);
    target_pose.pose.orientation.x = quat.x;
    target_pose.pose.orientation.y = quat.y;
    target_pose.pose.orientation.z = quat.z;
    target_pose.pose.orientation.w = quat.w;
    ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
    target_pose.pose.position.x = target_position_.x;
    target_pose.pose.position.y = target_position_.y;
    target_pose.pose.position.z = target_position_.z;

    group.setJointValueTarget(target_pose, group.getEndEffectorLink());
    group.setGoalTolerance(0.1);
    group.setGoalOrientationTolerance(0.01);
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

      // do non-blocking move request
      group.asyncExecute(plan);
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

    geometry_msgs::PoseStamped curr_pose_ = group.getCurrentPose();
    ROS_INFO("CURRENT POSE: x:%f y:%f z:%f", curr_pose_.pose.position.x,curr_pose_.pose.position.y,curr_pose_.pose.position.z);
    float distance = sqrt(pow(curr_pose_.pose.position.x-target_position_.x, 2) +
                          pow(curr_pose_.pose.position.y-target_position_.y, 2) +
                          pow(curr_pose_.pose.position.z-target_position_.z, 2) );
    ROS_INFO("Current distance to desired pose: %f", distance);

    //  TODO fix this condition as it needs to use some threshold
    //  now it accepts any distance
    if (distance < 0.02){
      going = false;
      success = true;
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