#include "motion/move_to_posture.hpp"


MoveToPostureAction::MoveToPostureAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&MoveToPostureAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&MoveToPostureAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting MoveToPosture server");
  as_.start();
}

MoveToPostureAction::~MoveToPostureAction(void) {
}

void MoveToPostureAction::init() {
  distance_threshold_ = 0.03;
}

void MoveToPostureAction::goalCB() {
  target_posture_ = as_.acceptNewGoal()->posture;
  this->executeCB();
}

void MoveToPostureAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void MoveToPostureAction::executeCB() {
  bool going = true;
  bool success = false;
  bool moveit_success = false;
  timed_out_ = false;
  // states:
  // 0 initial
  // 1 planned
  // 2 executing
  // 3 moved
  int state = 0;

  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);
  // get move it to execute motion
  moveit::planning_interface::MoveGroup group("arm_1");

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if (state == 0) {
      group.setNamedTarget(target_posture_);
      group.setPlanningTime(20.0);
      if (!group.plan(plan_)) {
        ROS_FATAL("Unable to create motion plan.  Aborting.");
        success = false;
        going = false;
      } else {
        ROS_INFO("Planning was successful");
        // Publish Feedback that plan was success
        feedback_.curr_state = 1;
        as_.publishFeedback(feedback_);
        state = 1;
      }
    } else if (state == 1) {
      // do blocking move request
      // ATTENTION: in simulation moveit may abort but continue the motion
      moveit_success = group.execute(plan_);
      // publish feedback that it is executing motion
      feedback_.curr_state = 2;
      as_.publishFeedback(feedback_);
      state = 2;
      timed_out_ = false;
      timer_ = nh_.createTimer(ros::Duration(30), &MoveToPostureAction::timerCB, this, true);
    } else if (state == 2) {
      // TODO: fix next lines
      success = true;
    }

    if (moveit_success) {
      going = false;
      success = true;
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

void MoveToPostureAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}
