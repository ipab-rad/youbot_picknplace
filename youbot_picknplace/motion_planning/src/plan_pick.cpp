#include "motion_planning/plan_pick.hpp"
#include <tf/transform_listener.h>


PlanPickAction::PlanPickAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  ac_gripper_("gripper_motion/move_gripper", true),
  ac_move_("motion/move_to_pose", true),
  ac_move_posture_("motion/move_to_posture", true),
  detect_ac_("sensing/object_detection", true)  {
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
  approach_dist_ = 0.05;
  min_grasp_dist_ = 0.3;
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
  bool init = true;
  // moving state
  bool moving = false;
  bool moving_gripper = false;

  // states:
  // 0 approach object
  // 1 open gripper
  // 2 approach object
  // 3 close gripper
  // 4 move away from object
  // 5 end
  int state = 0;
  int endstate = 6;
  ros::Rate r(10);

  ROS_INFO("Executing goal for %s", action_name_.c_str());

  int pick_attempts = 1;
  // motion attempts
  int motion_attempts = -1;

  geometry_msgs::PoseStamped gripper_pose;
  double distanceObj = -1.0;

  // start motion
  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }


    if (moving) {
      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Motion action success");

        state++;
        moving = false;
      } else if (ac_move_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("Motion action failed");

        if (motion_attempts > 0) {
          motion_attempts--;
        } else {
          going = false;
        }
        moving = false;
      }
    } else if (moving_gripper) {
      if (ac_gripper_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state++;
        ROS_INFO("Gripper action success");
        moving_gripper = false;
      } else  if (ac_gripper_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("Gripper action failed: %s", ac_gripper_.getState().toString().c_str());
        moving_gripper = false;
        // TODO: determine next state
      }
    } else if (init) {
      motion_attempts = 6;
      gripper_pose = object_pose_;
      gripper_pose.pose.orientation = computeGripperGraspPose(object_pose_.pose.position);

      distanceObj = sqrt(pow(0.143 - object_pose_.pose.position.x, 2) +
                         pow(object_pose_.pose.position.y, 2));
      ROS_INFO("Object 2D distance to base_link is: %f", distanceObj);

      // Safety guard for never attempting to reach below the ground
      double ground_limit = 0.095; // 9,5cm
      if (gripper_pose.pose.position.z < ground_limit) {
        ROS_INFO("Attempting to reach below ground limit %f with z-coord: %f", ground_limit, gripper_pose.pose.position.z);
        ROS_INFO("Assuming z='ground limit' for safety");
        gripper_pose.pose.position.z = ground_limit;
      }
      init = false;
    } else if (state == 0) {
      // target pose declaration
      geometry_msgs::PoseStamped target_pose = gripper_pose;
      // approach object
      target_pose.pose.position.z += approach_dist_;
      arm_goal_.pose = target_pose;
      arm_goal_.distance_tol = 0.01;
      arm_goal_.orientation_tol = 0.1;
      arm_goal_.planning_time = 20.0;
      ROS_INFO("Approaching object");
      ac_move_.sendGoal(arm_goal_);
      moving = true;

    } else if (state == 1) {
      // open gripper action
      gripper_goal_.command = 1;
      ROS_INFO("Opening Gripper");
      ac_gripper_.sendGoal(gripper_goal_);
      moving_gripper = true;

    } else if (state == 2) {
      // move to pose action
      arm_goal_.pose = gripper_pose;
      arm_goal_.distance_tol = 0.01;
      arm_goal_.orientation_tol = 0.1;
      arm_goal_.planning_time = 20.0;
      ROS_INFO("Making contact with object");
      ac_move_.sendGoal(arm_goal_);
      moving = true;

    } else if (state == 3) {
      // close gripper
      gripper_goal_.command = 0;
      ROS_INFO("Closing Gripper");
      ac_gripper_.sendGoal(gripper_goal_);
      moving_gripper = true;

    } else if (state == 4) {
      // moving away object
      geometry_msgs::PoseStamped target_pose = gripper_pose;
      target_pose.pose.position.z += approach_dist_;
      arm_goal_.pose = target_pose;
      arm_goal_.distance_tol = 0.02;
      arm_goal_.orientation_tol = 0.2;
      arm_goal_.planning_time = 20.0;
      ROS_INFO("Moving away from object");
      ac_move_.sendGoal(arm_goal_);
      moving = true;

    } else if (state == 5) {

      // move to detection position
      motion_msgs::MoveToPostureGoal posture_goal_;
      if (object_pose_.pose.position.y > 0.0)
        posture_goal_.posture = "check_left";
      else
        posture_goal_.posture = "check_right";

      if (object_pose_.pose.position.x > 0.3)
        posture_goal_.posture = "check_front";

      ROS_INFO("Checking for object with posture %s", posture_goal_.posture.c_str());
      ac_move_posture_.sendGoal(posture_goal_);
      // start detection action
      sensing_msgs::DetectObjectGoal detect_goal;
      detect_goal.detect = true;
      detect_goal.timeout = 10;
      detect_ac_.sendGoal(detect_goal);
      state = endstate;

    } else if (state == endstate) {
      if (detect_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        if (pick_attempts > 0) {
          pick_attempts--;
          object_pose_ = detect_ac_.getResult()->pose;
          init = true;
          state = 0;
        } else {
          going = false;
        }
      } else if (detect_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        success = true;
        going = false;
      }
    }

    ros::spinOnce();
    r.sleep();
  }

  if (success) {
    result_.success = 1;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    if (distanceObj > min_grasp_dist_) { // out of reach case
      result_.success = -2;
      result_.suggestion = computeSuggestedMovement(min_grasp_dist_, object_pose_.pose.position);
    } else
      result_.success = -1;

    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }
}

geometry_msgs::Quaternion computeGripperGraspPose(geometry_msgs::Point pt) {
  // orientation
  double base_trans = 0.143;
  double tan_angle = atan2(pt.y, pt.x - base_trans);
  ROS_INFO("Computed angle: %f", tan_angle);

  double offset = 0.25;
  double yaw_angle = tan_angle + offset;

  ROS_INFO("Desired Gripper RPY orientation: (%f,%f,%f)", 3.141, 0.001, yaw_angle);

  return tf::createQuaternionMsgFromRollPitchYaw(3.141, 0.001, yaw_angle);
}

geometry_msgs::Point computeSuggestedMovement(double min_grasp, geometry_msgs::Point pt) {
  // orientation
  geometry_msgs::Point result;
  double base_offset = 0.143;
  double dist = sqrt(pow(base_offset - pt.x, 2) + pow(pt.y, 2));

  double radius = dist - min_grasp + 0.02; // always move a bit more than needed
  double tan_angle = atan2(pt.y, pt.x - base_offset);
  ROS_INFO("Computed angle: %f", tan_angle);

  result.x = radius * cos(tan_angle);
  result.y = radius * sin(tan_angle);

  ROS_INFO("Suggested movement: (%f,%f,0.0)", result.x, result.y);

  return result;
}
