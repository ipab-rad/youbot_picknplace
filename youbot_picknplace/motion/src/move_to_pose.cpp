/**
 * @file      move_to_pose.cpp
 * @brief     Arm motion node. Uses end-effector goal specification.
 * @author    Alexandre Silva <s1464657@sms.ed.ac.uk>
 * @date      2015-08-20
 */

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
}

void MoveToPoseAction::goalCB() {
  boost::shared_ptr< const motion_msgs::MoveToPoseGoal > goal = as_.acceptNewGoal();
  target_pose_ = goal->pose;
  distance_tol_ = goal->distance_tol;
  planning_time_ = goal->planning_time;
  orientation_tol_ = goal->orientation_tol;
  grasping_move_ = goal->grasping_move;
  this->executeCB();
}

void MoveToPoseAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void MoveToPoseAction::executeCB() {
  bool going = true;
  bool success = false;
  bool plan_grasp_action = grasping_move_;
  bool moving = false;

  timed_out_ = false;
  // states:
  // 0 plan
  // 1 execute
  // 2 execute grasp action
  // 3 end
  int state = 0;
  int endstate = 3;

  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);
  // get move it to execute motion
  moveit::planning_interface::MoveGroup group("arm_1");
  // moveit motion plan
  move_group_interface::MoveGroup::Plan plan;
  move_group_interface::MoveGroup::Plan grasp_plan;
  // approach target pose
  double approach_dist = 0.05;
  geometry_msgs::PoseStamped target_pose = target_pose_;
  geometry_msgs::PoseStamped approach_pose = target_pose_;
  approach_pose.pose.position.z += approach_dist;

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // plan approach of a grasp action
    if (plan_grasp_action) {
      group.setPoseTarget(approach_pose, group.getEndEffectorLink());
      group.setGoalTolerance(distance_tol_);
      group.setGoalOrientationTolerance(orientation_tol_);
      group.setPlanningTime(planning_time_);
      // setup first target
      if (group.plan(plan)) {
        std::vector<double> init_state_ = plan.trajectory_.joint_trajectory.points.back().positions;
        robot_state::RobotState start_state(*group.getCurrentState());
        start_state.setVariablePosition(0, init_state_[0]);
        start_state.setVariablePosition(1, init_state_[1]);
        start_state.setVariablePosition(2, init_state_[2]);
        start_state.setVariablePosition(3, init_state_[3]);
        start_state.setVariablePosition(4, init_state_[4]);
        group.setStartState(start_state);
        group.setPoseTarget(target_pose_, group.getEndEffectorLink());
        group.setGoalTolerance(distance_tol_);
        group.setGoalOrientationTolerance(orientation_tol_);
        group.setPlanningTime(planning_time_);
        if (group.plan(grasp_plan)) {
          double base_joint_1 = init_state_[0];
          double base_joint_2 = grasp_plan.trajectory_.joint_trajectory.points.back().positions[0];

          if (fabs(base_joint_1 - base_joint_2) < 1.0) {
            plan_grasp_action = false;
            // have to set target pose as we will approach first
            target_pose = approach_pose;
            // go to execute plan
            state = 1;
          }
        }
      }

      // terminate if no plan
      if (state == 0)
        going = false;

      // terminate if no plan
      if (state == 0) {
        going = false;
        ROS_WARN("Failed planning smooth grasp trajectory");
      } else
        ROS_WARN("Valid grasp plan");

    } if (moving) {
      curr_pose_ = group.getCurrentPose();
      // ROS_INFO("CURRENT POSE: x:%f y:%f z:%f", curr_pose_.pose.position.x, curr_pose_.pose.position.y, curr_pose_.pose.position.z);
      distance_ = sqrt(pow(curr_pose_.pose.position.x - target_pose.pose.position.x, 2) +
                       pow(curr_pose_.pose.position.y - target_pose.pose.position.y, 2) +
                       pow(curr_pose_.pose.position.z - target_pose.pose.position.z, 2) );
      // ROS_INFO("Current distance to desired pose: %f", distance_);

      if (distance_ < distance_tol_) {
        if (grasping_move_)
          state++;
        else
          state = endstate;
        moving = false;
      }
    } else if (state == 0) {
      group.setPoseTarget(target_pose_, group.getEndEffectorLink());
      group.setGoalTolerance(distance_tol_);
      group.setGoalOrientationTolerance(orientation_tol_);
      group.setPlanningTime(planning_time_);
      if (!group.plan(plan)) {
        ROS_FATAL("Unable to create motion plan.  Aborting.");
        going = false;
      } else {
        ROS_INFO("Planning was successful");
        state = 1;
      }
    } else if (state == 1) {
      // do blocking move request
      // ATTENTION: moveit may abort but continue the motion
      if (group.execute(plan)) {
        if (grasping_move_) {
          state++;
        } else {
          state = endstate;
        }
      } else {
        // set timer
        // setting an action timer
        timed_out_ = false;
        timer_ = nh_.createTimer(ros::Duration(10), &MoveToPoseAction::timerCB, this, true);
        moving = true;
        ROS_INFO("Waiting to reach desired position...");
      }
    } else if (state == 2) {
      // execute second part of grasp action
      // do blocking move request
      // ATTENTION: moveit may abort but continue the motion
      target_pose = target_pose_;
      timer_.stop();
      if (group.execute(grasp_plan)) {
        state++;
      } else {
        // set timer
        // setting an action timer
        timed_out_ = false;
        timer_ = nh_.createTimer(ros::Duration(10), &MoveToPoseAction::timerCB, this, true);
        moving = true;
        ROS_INFO("Waiting to reach desired position...");
      }
    } else if (state == endstate) {
      going = false;
      success = true;
    }


    if (timed_out_) {
      ROS_INFO("%s: Timed out", action_name_.c_str());
      going = false;
    }

    ros::spinOnce();
    r.sleep();
  }


  curr_pose_ = group.getCurrentPose();
  ROS_INFO("TARGET POSE: x:%f y:%f z:%f", target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z);
  ROS_INFO("CURRENT POSE: x:%f y:%f z:%f", curr_pose_.pose.position.x, curr_pose_.pose.position.y, curr_pose_.pose.position.z);
  distance_ = sqrt(pow(curr_pose_.pose.position.x - target_pose_.pose.position.x, 2) +
                   pow(curr_pose_.pose.position.y - target_pose_.pose.position.y, 2) +
                   pow(curr_pose_.pose.position.z - target_pose_.pose.position.z, 2) );
  ROS_INFO("Current distance to desired pose: %f", distance_);
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
  timer_.stop();
}

void MoveToPoseAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}
