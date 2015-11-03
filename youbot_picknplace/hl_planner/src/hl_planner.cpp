/**
 * @file      hl_planner.cpp
 * @brief     High-Level Planner executes a list of action instructions
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2015-11-03
 * @copyright (MIT) 2015 RAD-UoE Informatics
 */

#include "hl_planner/hl_planner.hpp"

HLPlanner::HLPlanner(ros::NodeHandle* nh) :
  obj_ac_("/youbot_3/sensing/plan_detection", true),
  pick_ac_("/youbot_3/motion_planning/plan_pick", true),
  place_ac_("/youbot_3/motion_planning/plan_place", true) {
  nh_ = nh;
  this->loadParams();
  this->init();
  this->rosSetup();
}

HLPlanner::~HLPlanner() {
  ros::param::del("hl_planner");
}

void HLPlanner::init() {
  executing_ = false;
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
  ROS_INFO("Waiting for Place server to start.");
  place_ac_.waitForServer();
  place_goal_.place_object = true;
  ROS_INFO("Waiting for Pick server to start.");
  pick_ac_.waitForServer();
}

void HLPlanner::loadParams() {
  ros::param::get("/hl_planner/plan_path", plan_path_);
  ros::param::get("/hl_planner/plan_file", plan_file_);
  ros::param::get("/hl_planner/robot_state_", robot_state_);
  ros::param::get("/hl_planner/cube_a_state", cube_a_state_);
  ros::param::get("/hl_planner/cube_b_state", cube_b_state_);
}

void HLPlanner::execute() {
  if (!executing_) {
    for (size_t i = 0; i < action_list_.size(); ++i) {
      if (action_list_[i].compare(PICK_A) == 0) {
        ROS_INFO("Picking A!");
      } else if (action_list_[i].compare(PICK_B) == 0) {
        ROS_INFO("Picking B!");
      } else if (action_list_[i].compare(GOTO_A) == 0) {
        ROS_INFO("Goto A!");
      } else if (action_list_[i].compare(GOTO_B) == 0) {
        ROS_INFO("Goto B!");
      } else if (action_list_[i].compare(PLACE_A) == 0) {
        ROS_INFO("Place A!");
      } else if (action_list_[i].compare(PLACE_B) == 0) {
        ROS_INFO("Place B!");
      } else {
        ROS_ERROR("Action not recognised!");
      }
    }
  }
  executing_ = true;
}

void HLPlanner::pick_a() {
  pick_goal_.object_pose = cube_a_pose_;
  pick_ac_.sendGoal(pick_goal_);
  if (pick_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    executing_ = false;
  } else {failed_ = true;}
}

void HLPlanner::pick_b() {
  pick_goal_.object_pose = cube_b_pose_;
  pick_ac_.sendGoal(pick_goal_);
  if (pick_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    executing_ = false;
  } else {failed_ = true;}
}

void HLPlanner::goto_a() {
  if (robot_state_.compare("A") == 0) {
    ROS_WARN("Already at A!");
    executing_ = false;
  } else {
    // NAVIGATION ACTION
    // SENSING ACTION
  }
}

void HLPlanner::goto_b() {
  if (robot_state_.compare("B") == 0) {
    ROS_WARN("Already at B!");
    executing_ = false;
  } else {
    // NAVIGATION ACTION
    // SENSING ACTION
  }
}

void HLPlanner::place_a() {
  place_ac_.sendGoal(place_goal_);
}

void HLPlanner::place_b() {
  place_ac_.sendGoal(place_goal_);
}

bool HLPlanner::interrupted_ = false;

void HLPlanner::interrupt(int s) {
  HLPlanner::interrupted_ = true;
  ROS_INFO("HLPlanner Interrupted!");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hl_planner");
  ros::NodeHandle nh("hl_planner");
  HLPlanner hl_planner(&nh);
  ROS_INFO("High-Level Planner initialised");

  std::signal(SIGINT, HLPlanner::interrupt);

  ros::Rate r(10);

  while (ros::ok() && !HLPlanner::isInterrupted()) {
    ros::spinOnce();
    hl_planner.execute();
    r.sleep();
  }

  ros::shutdown();

  return 0;
}
