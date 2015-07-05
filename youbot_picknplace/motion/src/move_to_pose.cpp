#include "motion/move_to_pose.hpp"

MoveToPoseAction::MoveToPoseAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&MoveToPoseAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&MoveToPoseAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting MoveToPose server");
  as_.start();
}

MoveToPoseAction::~MoveToPoseAction(void) {
}

void MoveToPoseAction::init() {
  distance_threshold_ = 0.03;
}

void MoveToPoseAction::goalCB() {
  target_pose_ = as_.acceptNewGoal()->pose;
  this->executeCB();
}

void MoveToPoseAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void MoveToPoseAction::executeCB() {
  bool going = true;
  bool success = false;
  // states:
  // 0 initial
  // 1 planned
  // 2 moving
  // 3 moved
  int state = 0;

  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);
  // get move it to execute motion
  moveit::planning_interface::MoveGroup group("arm_1");

  // setting an action timer
  timed_out_ = false;
  timer_ = nh_.createTimer(ros::Duration(60), &MoveToPoseAction::timerCB, this, true);

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if (state == 0) {
      group.setJointValueTarget(target_pose_, group.getEndEffectorLink());
      group.setGoalTolerance(0.02);
      group.setGoalOrientationTolerance(0.02);
      group.setPlanningTime(20.0);
      if (!group.plan(plan)) {
        ROS_FATAL("Unable to create motion plan.  Aborting.");
        success = false;
        going = false;
      } else {
        ROS_INFO("Planning was successful");
        state = 1;
      }
    } else if (state == 1) {
      // Publish Feedback that plan was success
      feedback_.curr_state = 1;
      as_.publishFeedback(feedback_);

      // do blocking move request
      // ATTENTION: moveit may abort but continue the motion
      group.execute(plan);
      // publish feedback that it is executing motion
      feedback_.curr_state = 2;
      as_.publishFeedback(feedback_);
      state = 2;

    } else if (state == 2) {
      curr_pose_ = group.getCurrentPose();
      ROS_INFO("CURRENT POSE: x:%f y:%f z:%f", curr_pose_.pose.position.x, curr_pose_.pose.position.y, curr_pose_.pose.position.z);
      distance_ = sqrt(pow(curr_pose_.pose.position.x - target_pose_.pose.position.x, 2) +
                       pow(curr_pose_.pose.position.y - target_pose_.pose.position.y, 2) +
                       pow(curr_pose_.pose.position.z - target_pose_.pose.position.z, 2) );
      ROS_INFO("Current distance to desired pose: %f", distance_);

      //  TODO fix this condition as it needs to use some threshold
      //  now it accepts any distance
      if (distance_ < distance_threshold_) {
        state = 3;
        going = false;
        success = true;
      }
    }

    if (timed_out_) {
      ROS_INFO("%s: Timed out", action_name_.c_str());
      // TODO: set as preempted?
      going = false;
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

void MoveToPoseAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}
