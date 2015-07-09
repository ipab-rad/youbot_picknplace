#include "motion_planning/plan_pick.hpp"

PlanPickAction::PlanPickAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  ac_gripper_("gripper_motion/move_gripper", true),
  ac_move_("motion/move_to_pose", true) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanPickAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanPickAction::preemptCB, this));
  // start gripper pose service client
  pose_c_ = nh_.serviceClient<motion_msgs::GripperPose>("motion/gripper_pose");

  this->init();
  ROS_INFO("Starting PlanPick server");
  as_.start();
}

PlanPickAction::~PlanPickAction(void) {
}

void PlanPickAction::init() {
  ac_gripper_.waitForServer();
  ac_move_.waitForServer();
  ROS_INFO("Connected to gripper and arm movement servers");
}

void PlanPickAction::goalCB() {
  object_pose_ = as_.acceptNewGoal()->object_pose;
  this->executeCB();
}

void PlanPickAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void PlanPickAction::executeCB() {
  bool going = true;
  bool success = false;
  // states:
  // 0 initial
  // 1 approached
  // 2 opened gripper
  // 3 grasped
  // 4 closed gripper
  int state = 0;
  ros::Rate r(10);
  feedback_.curr_state = 1;
  as_.publishFeedback(feedback_);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  // target pose declaration
  geometry_msgs::PoseStamped target_pose = object_pose_;

  // get gripper pose to grasp object based on obj position
  motion_msgs::GripperPose srv;
  srv.request.position = object_pose_.pose.position;
  if (pose_c_.call(srv))
  {
    ROS_INFO("Received orientation from GripperPose");
    target_pose.pose.orientation = srv.response.orientation;
  }
  else
  {
    ROS_ERROR("Failed to call service GripperPose");
    going = false;
  }

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if (state == 0) {
      // approach object 
      target_pose.pose.position.z += 0.05;
      arm_goal_.pose = target_pose;
      ROS_INFO("Approaching object");
      ac_move_.sendGoal(arm_goal_);
      ac_move_.waitForResult();

      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 1;
        ROS_INFO("Approching action success");
      } else {
        ROS_INFO("Approching action failed: %s", ac_move_.getState().toString().c_str());
      }
    } else if (state == 1) {
      // open gripper action
      gripper_goal_.command = 1;
      ROS_INFO("Opening Gripper");
      ac_gripper_.sendGoal(gripper_goal_);
      ac_gripper_.waitForResult();
      if (ac_gripper_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 2;
        ROS_INFO("Opening gripper action success");
      } else {
        ROS_INFO("Opening gripper action failed: %s", ac_gripper_.getState().toString().c_str());
      }
    } else if (state == 2) {
      // move to pose action
      target_pose.pose.position.z -= 0.05;
      arm_goal_.pose = target_pose;
      ROS_INFO("Making contact with object");
      ac_move_.sendGoal(arm_goal_);
      ac_move_.waitForResult();
      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 3;
        ROS_INFO("Make contact action success");
      } else {
        ROS_INFO("Make contact action failed: %s", ac_move_.getState().toString().c_str());
      }
    } else if (state == 3) {
      // close gripper
      gripper_goal_.command = 0;
      ROS_INFO("Closing Gripper");
      ac_gripper_.sendGoal(gripper_goal_);
      ac_gripper_.waitForResult();
      if (ac_gripper_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 4;
        ROS_INFO("Closing gripper action success");
      } else {
        ROS_INFO("Closing gripper action failed: %s", ac_gripper_.getState().toString().c_str());
      }
    } else if (state == 4) {
      // moving away object
      target_pose.pose.position.z += 0.05;
      arm_goal_.pose = target_pose;
      ROS_INFO("Moving away from object");
      ac_move_.sendGoal(arm_goal_);
      ac_move_.waitForResult();

      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 5;
        ROS_INFO("Moving away from object action success");
      } else {
        ROS_INFO("Moving away from object action failed: %s", ac_move_.getState().toString().c_str());
      }
    } else if (state == 5) {
      success = true;
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