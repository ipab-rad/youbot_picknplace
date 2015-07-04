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
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  // approach object
  geometry_msgs::PoseStamped target_pose = object_pose_;
  target_pose.pose.position.z += 0.05;
  arm_goal_.pose = target_pose;
  ROS_INFO("Approaching object");
  ac_move_.sendGoal(arm_goal_);

  sleep(10.0);
  // TODO: if action not complete, abort


  // open gripper action
  gripper_goal_.command = 1;
  ROS_INFO("Opening Gripper");
  ac_gripper_.sendGoal(gripper_goal_);

  // publish feedback of execution
  // TODO give better feedback
  feedback_.curr_state = 1;
  as_.publishFeedback(feedback_);

  sleep(5.0);

  // TODO: if action not complete, abort

  // move to pose action
  arm_goal_.pose = object_pose_;

  ROS_INFO("Picking up object");
  ac_move_.sendGoal(arm_goal_);

  sleep(10.0);

  // TODO: if action not complete, abort


  //  close gripper
  gripper_goal_.command = 0;
  ROS_INFO("Closing Gripper");
  ac_gripper_.sendGoal(gripper_goal_);

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

void PlanPickAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}
