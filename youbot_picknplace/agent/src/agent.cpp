#include <ros/ros.h>
// actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// messages
#include <motion_planning_msgs/PlanObjectDetectionAction.h>
#include <motion_planning_msgs/PlanPlaceAction.h>
#include <motion_planning_msgs/PlanPickAction.h>
#include <motion_planning_msgs/PlanGoHomeAction.h>
#include <motion_planning_msgs/PlanListenAoiAction.h>
#include <motion_planning_msgs/PlanApproachObjectAction.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "agent");

  if (argc != 2) {
    ROS_INFO("Please provide one argument: 1=motion 0=no motion");
    return 0;
  }

  bool going = true;
  bool success = false;


  // states:
  // 0 listen area of interest
  // 1 object detection
  // 2 pick
  // 3 place
  // 4 move closer
  int state = 0;
  if (atoi(argv[1]) == 0)
    state = 1;

  char *s = std::getenv("ROBOT_NAME");
  // action lib clients
  actionlib::SimpleActionClient<motion_planning_msgs::PlanListenAoiAction> nav_ac(std::string(s) + "/motion_planning/plan_listen_aoi", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanObjectDetectionAction> obj_ac(std::string(s) + "/motion_planning/plan_detection", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPickAction> pick_ac(std::string(s) + "/motion_planning/plan_pick", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPlaceAction> place_ac(std::string(s) + "/motion_planning/plan_place", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanGoHomeAction> home_ac(std::string(s) + "/motion_planning/plan_go_home", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanApproachObjectAction> approach_ac(std::string(s) + "/motion_planning/plan_approach_object", true);

  // START

  while (going) {

    // NAVIGATION by AREA OF INTEREST LISTENING
    if (state == 0) {
      ROS_INFO("Waiting for area of interest action server to start.");
      // wait for the action server to start
      nav_ac.waitForServer(); //will wait for infinite time

      ROS_INFO("Area of interest Action server started, sending goal.");
      // send a goal to the action
      motion_planning_msgs::PlanListenAoiGoal nav_goal;
      nav_goal.find = true;

      nav_ac.sendGoal(nav_goal);

      nav_ac.waitForResult();
      if (nav_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        state = 1;
      else
        going = false;
    } else if (state == 1) {

      ROS_INFO("Waiting for Detect action server to start.");
      // wait for the action server to start
      obj_ac.waitForServer(); //will wait for infinite time

      ROS_INFO("Detect Action server started, sending goal.");
      // send a goal to the action
      motion_planning_msgs::PlanObjectDetectionGoal obj_goal;
      obj_goal.detect = true;

      obj_ac.sendGoal(obj_goal);

      obj_ac.waitForResult();
      if (obj_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        state = 2;
      else
        going = false;
    } else if (state == 2) {
      // PICK

      ROS_INFO("Waiting for Pick action server to start.");
      // wait for the action server to start
      pick_ac.waitForServer(); //will wait for infinite time

      ROS_INFO("Pick Action server started, sending goal.");
      // send a goal to the action
      motion_planning_msgs::PlanPickGoal pick_goal;

      pick_goal.object_pose = obj_ac.getResult()->pose;
      ROS_INFO("Object to pick is at (%f,%f).", pick_goal.object_pose.pose.position.x, pick_goal.object_pose.pose.position.y);
      pick_ac.sendGoal(pick_goal);

      pick_ac.waitForResult();
      if (pick_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        state = 3;
      else if (pick_ac.getResult()->success == -2) {
        ROS_INFO("Out of reach. Suggested movement from Pick (%f,%f,0.0)"
                 , pick_ac.getResult()->suggestion.x, pick_ac.getResult()->suggestion.y);
        state = 4;
      } else
        going = false;
    } else if (state == 3) {
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
      if (place_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        success = true;
      going = false;
    } else if (state == 4) {
      // APPROACH NEARBY (OUT OF REACH) OBJECT
      ROS_INFO("Waiting for Approach Object server to start.");
      // wait for the action server to start
      approach_ac.waitForServer(); //will wait for infinite time

      ROS_INFO("Approach Object Action server started, sending goal.");
      // send a goal to the action
      motion_planning_msgs::PlanApproachObjectGoal approach_goal;
      approach_goal.position = pick_ac.getResult()->suggestion;

      approach_ac.sendGoal(approach_goal);
      approach_ac.waitForResult();
      if (approach_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        state = 1;
      else
        going = false;
      ros::Duration(3).sleep(); // sleep to allow for camera feed to flush
    }
  }// end while going

  if (success) {
    ROS_INFO("Task successful!");
  } else {
    ROS_INFO("Task failed! At state: %d", state);
  }

  // DEFAULT BEHAVIOUR AT END OF EXECUTION
  // GO HOME
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
  if (home_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Done!");
  else
    ROS_INFO("Failed going to home positon");

  //exit
  return 0;
}
