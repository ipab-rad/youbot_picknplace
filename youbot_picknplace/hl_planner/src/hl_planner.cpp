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
  detect_ac_("/youbot_3/sensing/object_detection", true),
  move_ac_("/youbot_3/move_base", true) {
  nh_ = nh;
  this->loadParams();
  this->init();
  this->rosSetup();
}

HLPlanner::~HLPlanner() {
  ros::param::del("hl_planner");
}

void HLPlanner::init() {
  // DON'T TOUCH INITIALISATION FLAGS
  finished_ = false;
  // Setting up actions
  PICK_A = "PICK CubeA";
  PICK_B = "PICK CubeB";
  GOTO_A = "MOVE LocA";
  GOTO_B = "MOVE LocB";
  PLACE_A = "PLACE CubeA";
  PLACE_B = "PLACE CubeB";

  enable_alignment_ = true;

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
  ROS_INFO("Waiting for Move Base Action Server...");
  move_ac_.waitForServer();

  det_sub_ = nh_->subscribe("/recognized_object_array", 1000,
                            &HLPlanner::emptyCB, this);
  cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>("/youbot_3/cmd_vel", 1000);

  stop_msg_.linear.x = 0.0;
  stop_msg_.linear.y = 0.0;
  stop_msg_.linear.z = 0.0;
  stop_msg_.angular.x = 0.0;
  stop_msg_.angular.y = 0.0;
  stop_msg_.angular.z = 0.0;

  freq_ = 10;
  velocity_ = 0.2;
  move_time_ = 3.0;
  move_left_ = stop_msg_;
  move_left_.linear.y = velocity_;

  move_right_ = stop_msg_;
  move_right_.linear.y = -velocity_;

  wait_init_ = ros::Duration(30.0);
  wait_ = ros::Duration(10.0);
  wait_detect_ = ros::Duration(60.0);

  close_.command = 0; // Close gripper
  open_.command = 1; // Open gripper

  candle_.posture = "candle";
  place_.posture = "place_front";
  drop_a_.posture = "back_drop_left";
  drop_b_.posture = "back_drop_right";
  home_.posture = "home";
  detect_.detect = true;
  detect_.timeout = 30;

  move_base_msgs::MoveBaseGoal empty;
  empty.target_pose.header.frame_id = "/youbot_3/map";
  empty.target_pose.pose.position.z = 0.0f;
  empty.target_pose.pose.orientation.x = 0.0f;
  empty.target_pose.pose.orientation.y = 0.0f;

  move_a_ = empty;
  move_a_.target_pose.pose.position.x = -3.7f;
  move_a_.target_pose.pose.position.y = 1.6f;
  move_a_.target_pose.pose.orientation.z = 0.0f;
  move_a_.target_pose.pose.orientation.w = 1.0f;

  move_b_ = empty;
  move_b_.target_pose.pose.position.x = -6.0f;
  move_b_.target_pose.pose.position.y = 1.6f;
  move_b_.target_pose.pose.orientation.z = 0.99f;
  move_b_.target_pose.pose.orientation.w = 0.01f;

  float cube_offset = 0.3f;

  move_a_cube_a_ = move_a_;
  move_a_cube_a_.target_pose.pose.position.y -= cube_offset;

  move_a_cube_b_ = move_a_;
  move_a_cube_b_.target_pose.pose.position.y += cube_offset;

  move_b_cube_a_ = move_b_;
  move_b_cube_a_.target_pose.pose.position.y -= cube_offset;

  move_b_cube_b_ = move_b_;
  move_b_cube_b_.target_pose.pose.position.y += cube_offset;
}

void HLPlanner::loadParams() {
  ros::param::get("/hl_planner/plan_path", plan_path_);
  ros::param::get("/hl_planner/plan_file", plan_file_);
  ros::param::get("/hl_planner/log_path", log_path_);
  ros::param::get("/hl_planner/exp_name", exp_name_);
  ros::param::get("/hl_planner/robot_state", robot_state_);
  ros::param::get("/hl_planner/cube_a_state", cube_a_state_);
  ros::param::get("/hl_planner/cube_b_state", cube_b_state_);
  ROS_INFO_STREAM("Robot in " << robot_state_);
  ROS_INFO_STREAM("Cube A in " << cube_a_state_);
  ROS_INFO_STREAM("Cube B in " << cube_b_state_);
}

void HLPlanner::execute() {
  ROS_INFO("Preparing robot");
  this->initLogs();
  this->initRobot();
  ROS_INFO("Robot setup complete");
  for (size_t i = 0; i < action_list_.size(); ++i) {
    failed_ = false;
    if (action_list_[i].compare(PICK_A) == 0) {
      ROS_INFO("Pick A!");
      this->pickA();
    } else if (action_list_[i].compare(PICK_B) == 0) {
      ROS_INFO("Pick B!");
      this->pickB();
    } else if (action_list_[i].compare(GOTO_A) == 0) {
      ROS_INFO("Goto A!");
      this->gotoA();
    } else if (action_list_[i].compare(GOTO_B) == 0) {
      ROS_INFO("Goto B!");
      this->gotoB();
    } else if (action_list_[i].compare(PLACE_A) == 0) {
      ROS_INFO("Place A!");
      this->placeA();
    } else if (action_list_[i].compare(PLACE_B) == 0) {
      ROS_INFO("Place B!");
      this->placeB();
    } else {
      ROS_ERROR("Action not recognised!");
    }
    this->updateLog(action_list_[i]);
    if (failed_) {
      ROS_WARN("Experiment failed!");
      break;
    }
  }
  ROS_INFO("Finished!");
  this->stopRobot();
  this->endRobot();
  finished_ = true;
}

void HLPlanner::initLogs() {
  struct stat st = {0};
  if (stat((log_path_ + exp_name_).c_str(), &st) == -1) {
    int err = mkdir((log_path_ + exp_name_).c_str(), 0700);
    if (err == -1) {ROS_ERROR("COULD NOT CREATE LOG FOLDER!");}
  }
  this->createStateFile();
  std::stringstream ss;
  ss << log_path_ + exp_name_ + "/log.dat";
  std::string file = ss.str();
  log_file_.open(file.c_str());
}

void HLPlanner::createStateFile() {
  std::stringstream ss;
  ss << log_path_ + exp_name_ + "/exp.state";
  std::string file = ss.str();
  state_file_.open(file.c_str());
  state_file_ << "(AND (RobotAt Loc" + robot_state_ + ") ";
  state_file_ << "(CubeAt CubeA Loc" + cube_a_state_ + ") ";
  state_file_ << "(CubeAt CubeB Loc" + cube_b_state_ + "))\n";
  state_file_.close();
}

void HLPlanner::updateLog(std::string action) {
  std::string fail;
  if (failed_) {fail = "true";} else {fail = "false";}
  log_file_ << "((" + action + ")\n";
  log_file_ << " (AND (RobotAt Loc" + robot_state_ + ")";
  log_file_ << " (CubeAt CubeA Loc" + cube_a_state_ + ")";
  log_file_ << " (CubeAt CubeB Loc" + cube_b_state_ + ")";
  log_file_ << " (ActionFailed {" + fail + "})))\n";
}

void HLPlanner::closeLog() {
  log_file_.close();
}

void HLPlanner::pickA() {
  if (robot_state_.compare(cube_a_state_) != 0) {
    ROS_WARN_STREAM("Cube A not in " << robot_state_ << "!");
    failed_ = true;
  } else {
    // Move to Cube A location
    if (enable_alignment_) {this->moveLeft();}
    // Pickup Cube A
    posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
    this->startDetection();
    detect_ac_.sendGoalAndWait(detect_, wait_detect_, wait_detect_);
    pick_.object_pose = detect_ac_.getResult()->pose;
    pick_ac_.sendGoalAndWait(pick_);
    this->stopDetection();
    if (pick_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      this->dropA();
      cube_a_state_ = "R";
    } else {failed_ = true;}
    if (enable_alignment_) {this->moveRight();}
  }
}

void HLPlanner::pickB() {
  if (robot_state_.compare(cube_b_state_) != 0) {
    ROS_WARN_STREAM("Cube B not in " << robot_state_ << "!");
    failed_ = true;
  } else {
    // Move to Cube B
    if (enable_alignment_) {this->moveRight();}
    // Pickup Cube B
    posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
    this->startDetection();
    detect_ac_.sendGoalAndWait(detect_, wait_detect_, wait_detect_);
    pick_.object_pose = detect_ac_.getResult()->pose;
    pick_ac_.sendGoalAndWait(pick_);
    this->stopDetection();
    if (pick_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      this->dropB();
      cube_b_state_ = "R";
    } else {failed_ = true;}
    if (enable_alignment_) {this->moveLeft();}
  }
}

void HLPlanner::gotoA() {
  if (robot_state_.compare("A") == 0) {
    ROS_WARN("Already at A!");
    failed_ = true;
  } else {
    posture_ac_.sendGoal(home_);
    move_ac_.sendGoalAndWait(move_a_);
    if (move_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("ROS_NAV: Arrived!");
      robot_state_ = "A";
    } else if (move_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("ROS_NAV: Aborted!");
      failed_ = true;
    }
    this->stopRobot();
  }
}

void HLPlanner::gotoB() {
  if (robot_state_.compare("B") == 0) {
    ROS_WARN("Already at B!");
    failed_ = true;
  } else {
    posture_ac_.sendGoal(home_);
    move_ac_.sendGoalAndWait(move_b_);
    if (move_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("ROS_NAV: Arrived!");
      robot_state_ = "B";
    } else if (move_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_WARN("ROS_NAV: Aborted!");
      failed_ = true;
    }
    this->stopRobot();
  }
}

void HLPlanner::placeA() {
  if (cube_a_state_.compare("R") != 0) {
    ROS_WARN("Cube A not on robot!");
    failed_ = true;
  } else {
    if (enable_alignment_) {this->moveLeft();}
    gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
    posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
    posture_ac_.sendGoalAndWait(drop_a_, wait_, wait_);
    gripper_ac_.sendGoalAndWait(close_, wait_, wait_);
    posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
    posture_ac_.sendGoalAndWait(place_, wait_, wait_);
    gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
    // TODO: Check that placing was successful!
    cube_a_state_ = robot_state_;
    posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
    if (enable_alignment_) {this->moveRight();}
  }
}

void HLPlanner::placeB() {
  if (cube_b_state_.compare("R") != 0) {
    ROS_WARN("Cube B not on robot!");
    failed_ = true;
  } else {
    if (enable_alignment_) {this->moveRight();}
    gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
    posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
    posture_ac_.sendGoalAndWait(drop_b_, wait_, wait_);
    gripper_ac_.sendGoalAndWait(close_, wait_, wait_);
    posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
    posture_ac_.sendGoalAndWait(place_, wait_, wait_);
    gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
    // TODO: Check that placing was successful!
    cube_b_state_ = robot_state_;
    posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
    if (enable_alignment_) {this->moveLeft();}
  }
}

void HLPlanner::dropA() {
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  posture_ac_.sendGoalAndWait(drop_a_, wait_, wait_);
  gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
}

void HLPlanner::dropB() {
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
  posture_ac_.sendGoalAndWait(drop_b_, wait_, wait_);
  gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
  posture_ac_.sendGoalAndWait(candle_, wait_, wait_);
}

void HLPlanner::startDetection() {
  ROS_INFO("Starting detection");
  system("/home/ubuntu12/Git/youbot-ws/src/youbot_picknplace/youbot_picknplace/hl_planner/src/start_detection.sh");

  // Wait until detection is fully working
  ros::Rate r(freq_);
  while (ros::ok() && det_sub_.getNumPublishers() < 1) {
    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("Detection ready");
}

void HLPlanner::stopDetection() {
  ROS_INFO("Stopping detection");
  system("/home/ubuntu12/Git/youbot-ws/src/youbot_picknplace/youbot_picknplace/hl_planner/src/stop_detection.sh");

  // Wait until detection is fully stopped
  ros::Rate r(freq_);
  while (ros::ok() && det_sub_.getNumPublishers() > 0) {
    ros::spinOnce();
    r.sleep();
  }
  ROS_INFO("Detection stopped");
}

void HLPlanner::moveLeft() {
  ros::Rate r(freq_);
  for (int i = 0; i < (freq_ * move_time_); ++i) {
    cmd_vel_pub_.publish(move_left_);
    ros::spinOnce();
    r.sleep();
  }
  cmd_vel_pub_.publish(stop_msg_);
  ros::spinOnce();
}

void HLPlanner::moveRight() {
  ros::Rate r(freq_);
  for (int i = 0; i < (freq_ * move_time_); ++i) {
    cmd_vel_pub_.publish(move_right_);
    ros::spinOnce();
    r.sleep();
  }
  cmd_vel_pub_.publish(stop_msg_);
  ros::spinOnce();
}

void HLPlanner::stopRobot() {
  cmd_vel_pub_.publish(stop_msg_);
  ros::spinOnce();
}

void HLPlanner::initRobot() {
  posture_ac_.sendGoalAndWait(home_, wait_init_, wait_init_);
  gripper_ac_.sendGoalAndWait(open_, wait_, wait_);
}

void HLPlanner::endRobot() {
  this->closeLog();
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
