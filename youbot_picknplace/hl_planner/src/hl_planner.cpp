/**
 * @file      hl_planner.cpp
 * @brief     High-Level Planner executes a list of action instructions
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-11-03
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "hl_planner/hl_planner.hpp"

HLPlanner::HLPlanner(ros::NodeHandle* nh) :
  posture_ac_("/youbot_3/motion/move_to_posture", true),
  gripper_ac_("/youbot_3/gripper_motion/move_gripper", true),
  pick_ac_("/youbot_3/motion_planning/plan_pick", true),
  detect_ac_("/youbot_3/sensing/object_detection", true) {
  nh_ = nh;
  this->loadParams();
  this->init();
  this->rosSetup();
}

HLPlanner::~HLPlanner() {
  ros::param::del("hl_planner");
}

void HLPlanner::init() {
  finished_ = false;
  failed_ = false;
  // Setting up actions
  PICK_A = "PICK_A";
  PICK_B = "PICK_B";
  GOTO_A = "GOTO_A";
  GOTO_B = "GOTO_B";
  PLACE_A = "PLACE_A";
  PLACE_B = "PLACE_B";

  // Read action list
  std::ifstream file((plan_path_ + plan_file_).c_str());
  std::string line;
  for ( std::string line; getline(file, line); ) {
    ROS_INFO_STREAM(line);
    action_list_.push_back(line);
  }
}

void HLPlanner::rosSetup() {
  ROS_INFO("Waiting for Move To Posture server to start.");
  posture_ac_.waitForServer();
  ROS_INFO("Waiting for Gripper server to start.");
  gripper_ac_.waitForServer();
  ROS_INFO("Waiting for Pick server to start.");
  pick_ac_.waitForServer();
  ROS_INFO("Waiting for Detect Object server to start.");
  detect_ac_.waitForServer();

  wait_init_ = ros::Duration(30.0);
  wait_ = ros::Duration(10.0);
  wait_detect_ = ros::Duration(5.0);

  close_.command = 0; // Close gripper
  open_.command = 1; // Open gripper

  candle_.posture = "candle";
  place_.posture = "place_front";
  drop_a_.posture = "back_drop_left";
  drop_b_.posture = "back_drop_right";
  home_.posture = "home";
  detect_.detect = true;
  detect_.timeout = 5;
}

void HLPlanner::loadParams() {
  ros::param::get("/hl_planner/plan_path", plan_path_);
  ros::param::get("/hl_planner/plan_file", plan_file_);
  ros::param::get("/hl_planner/robot_state_", robot_state_);
  ros::param::get("/hl_planner/cube_a_state", cube_a_state_);
  ros::param::get("/hl_planner/cube_b_state", cube_b_state_);
}

void HLPlanner::execute() {
  ROS_INFO("Preparing robot");
  this->initRobot();
  ROS_INFO("Robot setup complete");
  for (size_t i = 0; i < action_list_.size(); ++i) {
    if (action_list_[i].compare(PICK_A) == 0) {
      ROS_INFO("Pick A!");
      this->pick_a();
    } else if (action_list_[i].compare(PICK_B) == 0) {
      ROS_INFO("Pick B!");
      this->pick_b();
    } else if (action_list_[i].compare(GOTO_A) == 0) {
      ROS_INFO("Goto A!");
      this->goto_a();
    } else if (action_list_[i].compare(GOTO_B) == 0) {
      ROS_INFO("Goto B!");
      this->goto_b();
    } else if (action_list_[i].compare(PLACE_A) == 0) {
      ROS_INFO("Place A!");
      this->place_a();
    } else if (action_list_[i].compare(PLACE_B) == 0) {
      ROS_INFO("Place B!");
      this->place_b();
    } else {
      ROS_ERROR("Action not recognised!");
    }
    if (failed_) {
      ROS_WARN("Experiment failed!");
      break;
    }
  }
  ROS_INFO("Finished!");
  this->endRobot();
  finished_ = true;
}

void HLPlanner::pick_a() {
  // Move to Cube A location
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  detect_ac_.sendGoalAndWait(detect_, wait_detect_, wait_detect_);
  pick_.object_pose = detect_ac_.getResult()->pose;
  pick_ac_.sendGoalAndWait(pick_);
  if (pick_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    this->drop_a();
  } else {failed_ = true;}
}

void HLPlanner::pick_b() {
  // Move to Cube B location
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  detect_ac_.sendGoalAndWait(detect_, wait_detect_, wait_detect_);
  pick_.object_pose = detect_ac_.getResult()->pose;
  pick_ac_.sendGoalAndWait(pick_);
  if (pick_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    this->drop_b();
  } else {failed_ = true;}
}

void HLPlanner::goto_a() {
  if (robot_state_.compare("A") == 0) {
    ROS_WARN("Already at A!");
  } else {
    // NAVIGATION ACTION
  }
}

void HLPlanner::goto_b() {
  if (robot_state_.compare("B") == 0) {
    ROS_WARN("Already at B!");
  } else {
    // NAVIGATION ACTION
  }
}

void HLPlanner::place_a() {
  gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  posture_ac_.sendGoalAndWait(drop_a_, wait_, wait_);
  gripper_ac_.sendGoalAndWait(close_, wait_, wait_);
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  posture_ac_.sendGoalAndWait(place_, wait_, wait_);
  gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
}

void HLPlanner::place_b() {
  gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  posture_ac_.sendGoalAndWait(drop_b_, wait_, wait_);
  gripper_ac_.sendGoalAndWait(close_, wait_, wait_);
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  posture_ac_.sendGoalAndWait(place_, wait_, wait_);
  gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
}

void HLPlanner::drop_a() {
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  posture_ac_.sendGoalAndWait(drop_a_, wait_, wait_);
  gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
}

void HLPlanner::drop_b() {
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  posture_ac_.sendGoalAndWait(drop_b_, wait_, wait_);
  gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
}

void HLPlanner::initRobot() {
  posture_ac_.sendGoalAndWait(candle_, wait_init_, wait_init_);
  gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
}

void HLPlanner::endRobot() {
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  posture_ac_.sendGoalAndWait(home_, wait_, wait_);
}

bool HLPlanner::interrupted_ = false;

void HLPlanner::interrupt(int s) {
  HLPlanner::interrupted_ = true;
  ROS_INFO("HLPlanner Interrupted!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hl_planner");
  ros::NodeHandle nh("hl_planner");
  HLPlanner hl_planner(&nh);
  std::signal(SIGINT, HLPlanner::interrupt);
  ROS_INFO("High-Level Planner initialised");
  ros::Rate r(10);

  while (ros::ok() && !HLPlanner::isInterrupted() && !hl_planner.isFinished()) {
    ros::spinOnce();
    hl_planner.execute();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
