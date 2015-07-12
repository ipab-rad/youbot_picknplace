#include "motion_planning/plan_pick.hpp"
#include <tf/transform_listener.h>


PlanPickAction::PlanPickAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  ac_gripper_("gripper_motion/move_gripper", true),
  ac_move_("motion/move_to_pose", true),
  ac_move_posture_("motion/move_to_posture", true)  {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanPickAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanPickAction::preemptCB, this));

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
  int state = 1;
  ros::Rate r(10);
  feedback_.curr_state = 1;
  as_.publishFeedback(feedback_);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  int direction = -1;
  if (object_pose_.pose.position.x > 0.2) {
    // face forward
    direction = 2;
  } else if (object_pose_.pose.position.y > 0.0) {
    // face left
    direction = 1;
  } else {
    // face right
    direction = 3;
  }

  geometry_msgs::PoseStamped gripper_pose = object_pose_;
  gripper_pose.pose.orientation = computeGripperGraspPose(object_pose_.pose.orientation);

  // Safety guard for never attempting to reach below the ground
  if (gripper_pose.pose.position.z < 0.02) {
    ROS_INFO("Attempting to reach below the ground with z-coord: %f", gripper_pose.pose.position.z);
    ROS_INFO("Assuming z=0.02 for safety");
    gripper_pose.pose.position.z = 0.02;
  }


  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // if (state == 0) {
    //   // initial base alignment
    //   if (direction == 1)
    //     arm_posture_goal_.posture = "l_pre_grasp";
    //   else if (direction == 2)
    //     arm_posture_goal_.posture = "f_pre_grasp";
    //   else
    //     arm_posture_goal_.posture = "r_pre_grasp";

    //   // move to pose action
    //   ac_move_posture_.waitForServer();
    //   ROS_INFO("Initial Alignment");

    //   ac_move_posture_.sendGoal(arm_posture_goal_);
    //   feedback_.curr_state = 1;
    //   as_.publishFeedback(feedback_);
    //   ac_move_posture_.waitForResult();
    //   if (ac_move_posture_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //     state = 1;
    //   }
    // } else
    if (state == 1) {
      // target pose declaration
      geometry_msgs::PoseStamped target_pose = gripper_pose;

      // approach object
      target_pose.pose.position.z += 0.05;
      arm_goal_.pose = target_pose;
      ROS_INFO("Approaching object");
      ac_move_.sendGoal(arm_goal_);
      ac_move_.waitForResult();

      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 2;
        ROS_INFO("Approching action success");
      } else {
        ROS_INFO("Approching action failed: %s", ac_move_.getState().toString().c_str());
        // going = false;
      }
    } else if (state == 2) {
      // open gripper action
      gripper_goal_.command = 1;
      ROS_INFO("Opening Gripper");
      ac_gripper_.sendGoal(gripper_goal_);
      ac_gripper_.waitForResult();
      if (ac_gripper_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 3;
        ROS_INFO("Opening gripper action success");
      } else {
        ROS_INFO("Opening gripper action failed: %s", ac_gripper_.getState().toString().c_str());
      }
    } else if (state == 3) {
      // move to pose action
      arm_goal_.pose = gripper_pose;
      ROS_INFO("Making contact with object");
      ac_move_.sendGoal(arm_goal_);
      ac_move_.waitForResult();
      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 4;
        ROS_INFO("Make contact action success");
      } else {
        ROS_INFO("Make contact action failed: %s", ac_move_.getState().toString().c_str());
        // going = false;
      }
    } else if (state == 4) {
      // close gripper
      gripper_goal_.command = 0;
      ROS_INFO("Closing Gripper");
      ac_gripper_.sendGoal(gripper_goal_);
      ac_gripper_.waitForResult();
      if (ac_gripper_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 5;
        ROS_INFO("Closing gripper action success");
      } else {
        ROS_INFO("Closing gripper action failed: %s", ac_gripper_.getState().toString().c_str());
      }
    } else if (state == 5) {
      // moving away object
      geometry_msgs::PoseStamped target_pose = gripper_pose;
      target_pose.pose.position.z += 0.05;
      arm_goal_.pose = target_pose;
      ROS_INFO("Moving away from object");
      ac_move_.sendGoal(arm_goal_);
      ac_move_.waitForResult();

      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 6;
        ROS_INFO("Moving away from object action success");
      } else {
        ROS_INFO("Moving away from object action failed: %s", ac_move_.getState().toString().c_str());
      }
    } else if (state == 6) {
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

geometry_msgs::Quaternion computeGripperGraspPose(geometry_msgs::Quaternion quat) {
  // orientation
  tf::Matrix3x3 mat(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  double roll; double pitch; double yaw;
  mat.getRPY(roll, pitch, yaw);
  ROS_INFO("Cube RPY orientation: (%f,%f,%f)", roll, pitch, yaw);

  double offset = 1.57; // pi/2
  double yaw_angle = yaw;

  if (yaw < 0.0)
    yaw_angle += offset;
  else
    yaw_angle -= 3 * offset;

  ROS_INFO("Desired Gripper RPY orientation: (%f,%f,%f)", 3.14, 0.0, yaw_angle);

  return tf::createQuaternionMsgFromRollPitchYaw(3.14, 0.0, yaw_angle);
}
