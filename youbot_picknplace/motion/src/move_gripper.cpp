#include "motion/move_gripper.hpp"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "brics_actuator/JointPositions.h"
#include <boost/units/systems/si/length.hpp>
#include <boost/units/io.hpp>


MoveGripperAction::MoveGripperAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&MoveGripperAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&MoveGripperAction::preemptCB, this));

  // use gripper topic to send commands
  gripper_topic_ = "/arm_1/gripper_controller/position_command";
  gripper_topic_sim_ = "/arm_1/gripper_controller/command";
  gripper_pub_ =
    nh_.advertise<brics_actuator::JointPositions>(gripper_topic_, 10, true);
  gripper_pub_sim_ =
    nh_.advertise<trajectory_msgs::JointTrajectory>(gripper_topic_sim_, 10, true);

  joint_name1_ = "gripper_finger_joint_l";
  joint_name2_ = "gripper_finger_joint_r";

  this->init();
  ROS_INFO("Starting MoveGripper server");
  as_.start();
}

MoveGripperAction::~MoveGripperAction(void) {
}

void MoveGripperAction::init() {
}

void MoveGripperAction::goalCB() {
  gripper_command_ = as_.acceptNewGoal()->command;
  this->executeCB();
}

void MoveGripperAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void MoveGripperAction::executeCB() {
  bool going = true;
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  bool sim = gripper_pub_.getNumSubscribers() == 0;
  ROS_INFO("We are in simulation mode: %d", sim);
  // if we are on real robot
  if (!sim) {
    // create trajectory message for gripper
    brics_actuator::JointPositions command;

    std::vector <brics_actuator::JointValue> gripperJointPositions;
    gripperJointPositions.resize(2);
    gripperJointPositions[0].joint_uri = joint_name1_;
    gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);
    gripperJointPositions[1].joint_uri = joint_name2_;
    gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);

    if (gripper_command_ == 1) { // open
      gripperJointPositions[0].value = 0.0109;
      gripperJointPositions[1].value = 0.0109;

    } else { // close
      gripperJointPositions[0].value = 0.0011;
      gripperJointPositions[1].value = 0.0011;
    }

    command.positions = gripperJointPositions;
    gripper_pub_.publish(command);
  } else { // we are in simulation
    // create trajectory message for gripper
    trajectory_ = trajectory_msgs::JointTrajectory();
    trajectory_.header.frame_id = "0";

    // gripper joint names
    trajectory_.joint_names.push_back(joint_name1_);
    trajectory_.joint_names.push_back(joint_name2_);

    // TODO: fix here position of points based on command
    trajectory_pt1_ = trajectory_msgs::JointTrajectoryPoint();

    if (gripper_command_ == 1) { // open
      trajectory_pt1_.positions.push_back(0.0109);
      trajectory_pt1_.positions.push_back(0.0109);
    } else { // close
      trajectory_pt1_.positions.push_back(0.0011);
      trajectory_pt1_.positions.push_back(0.0011);
    }
    ros::Duration duration(1, 0);
    trajectory_pt1_.time_from_start = duration;
    trajectory_.points.push_back(trajectory_pt1_);

    gripper_pub_sim_.publish(trajectory_);
  }
  // TODO: fix next lines
  going = false;
  success = true;


  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
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


int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_motion");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("Running Gripper motion");
  MoveGripperAction movegripper(nh, "move_gripper");


  ros::spin();
  return 0;
}

