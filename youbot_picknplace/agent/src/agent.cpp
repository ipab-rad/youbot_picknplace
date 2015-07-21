#include <ros/ros.h>
// actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// messages
#include <navigation_msgs/MoveToPositionAction.h>
#include <motion_planning_msgs/PlanObjectDetectionAction.h>
#include <motion_planning_msgs/PlanPlaceAction.h>
#include <motion_planning_msgs/PlanPickAction.h>
#include <motion_planning_msgs/PlanGoHomeAction.h>
//tf
#include <tf/transform_datatypes.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "agent");

  if(argc!=2){
    ROS_INFO("Please provide one argument: 1=motion 0=no motion");
    return 0;
  }


  bool done_nav = false;
  if (atoi(argv[1])==0)
    done_nav = true;

  bool done_detect = false;
  bool done_pick = false;
  bool done_place = false;
  bool done_home = false;
  // action lib clients
  actionlib::SimpleActionClient<navigation_msgs::MoveToPositionAction> nav_ac("navigation/move_to_position", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanObjectDetectionAction> obj_ac("motion_planning/plan_detection", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPickAction> pick_ac("motion_planning/plan_pick", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPlaceAction> place_ac("motion_planning/plan_place", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanGoHomeAction> home_ac("motion_planning/plan_go_home", true);
  
  // START

  // NAVIGATION
  if(!done_nav){
    ROS_INFO("Waiting for navigation action server to start.");
    // wait for the action server to start
    nav_ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Move to Position Action server started, sending goal.");
    // send a goal to the action
    navigation_msgs::MoveToPositionGoal nav_goal;
    geometry_msgs::Point position;
    position.x = 0.75;
    position.y = 0.0;
    position.z = 0.0;
    nav_goal.position = position;
    nav_goal.relative = true;

    nav_ac.sendGoal(nav_goal);

    nav_ac.waitForResult();
    if (nav_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      done_nav = true;
    }
  }

  // OBJECT DETECTION
  if(done_nav){
    ROS_INFO("Waiting for Detect action server to start.");
    // wait for the action server to start
    obj_ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Detect Action server started, sending goal.");
    // send a goal to the action
    motion_planning_msgs::PlanObjectDetectionGoal obj_goal;
    obj_goal.detect = true;

    obj_ac.sendGoal(obj_goal);

    obj_ac.waitForResult();
    if (obj_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      done_detect = true;
    }
  }

  if (done_detect) {
    // PICK

    ROS_INFO("Waiting for Pick action server to start.");
    // wait for the action server to start
    pick_ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Pick Action server started, sending goal.");
    // send a goal to the action
    motion_planning_msgs::PlanPickGoal pick_goal;

    // TEST
    // geometry_msgs::PoseStamped target_pose;
    // target_pose.header.frame_id = "base_footprint";
    // geometry_msgs::Quaternion quat;
    // quat = tf::createQuaternionMsgFromRollPitchYaw(-3.129, 0.0549, 1.686);
    // target_pose.pose.orientation.x = quat.x;
    // target_pose.pose.orientation.y = quat.y;
    // target_pose.pose.orientation.z = quat.z;
    // target_pose.pose.orientation.w = quat.w;
    // ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
    // target_pose.pose.poition.x = 0.12;
    // target_pose.pose.position.y = -0.25;
    // target_pose.pose.position.z = 0.0;
    // pick_goal.object_pose = target_pose;

    pick_goal.object_pose = obj_ac.getResult()->pose;

    pick_ac.sendGoal(pick_goal);

    pick_ac.waitForResult();
    if (pick_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      done_pick = true;
    }
  }


  if (done_pick) {
    // PLACE
    ROS_INFO("Waiting for Place server to start.");
    // wait for the action server to start
    place_ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Place Action server started, sending goal.");
    // send a goal to the action
    motion_planning_msgs::PlanPlaceGoal place_goal;
    place_goal.place_object = true;


    place_ac.sendGoal(place_goal);
    place_ac.waitForResult();
    if (place_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      done_place = true;
    }
  }

  if (done_place) {
    ROS_INFO("Task successful!");
  } else {
    ROS_INFO("Task failed!");
  }

  // PLACE
  ROS_INFO("Waiting for Go home server to start.");
  // wait for the action server to start
  home_ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Go Home Action server started, sending goal.");
  ROS_INFO("Goint to home pose");
  // send a goal to the action
  motion_planning_msgs::PlanGoHomeGoal home_goal;
  home_goal.go_home = true;


  home_ac.sendGoal(home_goal);
  home_ac.waitForResult();
  if (home_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    done_home = true;
  }


  if (done_home) {
    ROS_INFO("Done!");
  } else {
    ROS_INFO("Failed going to home positon");
  }

  //exit
  return 0;
}
