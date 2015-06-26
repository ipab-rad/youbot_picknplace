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
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  // Do a planning request
  planning_srv_.request.x = target_position_.x;
  planning_srv_.request.y = target_position_.y;
  planning_srv_.request.z = target_position_.z;
  

  // calling service
  if (planning_client_.call(planning_srv_)) {
    ROS_INFO("Reached planning service %s", action_name_.c_str());
    if(planning_srv_.response.success){
      ROS_INFO("Planning was successful");
      // Publish Feedback
      feedback_.curr_state = 1;
      as_.publishFeedback(feedback_);
    }else{
      ROS_INFO("Planning was NOT successful");
      success = false;
      going = false;
    }
   }else{
    ROS_INFO("Planning service %s was not reached", action_name_.c_str());
     success = false;
    as_.setPreempted();
     going = false;
   }

  if (going){
    feedback_.curr_state = 2;
    as_.publishFeedback(feedback_);
    // get move it to execute motion
    moveit::planning_interface::MoveGroup group("arm_1");
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "base_footprint";
    geometry_msgs::Quaternion quat ;
    quat = tf::createQuaternionMsgFromRollPitchYaw(3.1174,-0.1087,-1.6135);
    target_pose.pose.orientation.x = quat.x;
    target_pose.pose.orientation.y = quat.y;
    target_pose.pose.orientation.z = quat.z;
    target_pose.pose.orientation.w = quat.w;
    ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
    target_pose.pose.position.x = target_position_.x;
    target_pose.pose.position.y = target_position_.y;
    target_pose.pose.position.z = target_position_.z;
    group.setPoseTarget(target_pose, group.getEndEffectorLink());
    group.setGoalTolerance(0.1);
    group.setGoalOrientationTolerance(0.01);
    group.setPlanningTime(20.0);

    // Moving to pose goal
    group.move();
    ROS_INFO("Commanded Move Group to execute motion");
  }

  
  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    moveit::planning_interface::MoveGroup group("arm_1");

    geometry_msgs::PoseStamped curr_pose_ = group.getCurrentPose();
    ROS_INFO("CURRENT POSE: x:%f y:%f z:%f", curr_pose_.pose.position.x,curr_pose_.pose.position.y,curr_pose_.pose.position.z);
    float distance = sqrt(pow(curr_pose_.pose.position.x-target_position_.x, 2) +
                          pow(curr_pose_.pose.position.y-target_position_.y, 2) +
                          pow(curr_pose_.pose.position.z-target_position_.z, 2) );
    ROS_INFO("Current distance to desired pose: %f", distance);



    //  TODO fix this condition as it needs to use some threshold
    //  now it accepts any distance
    if (distance < 0.1){
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