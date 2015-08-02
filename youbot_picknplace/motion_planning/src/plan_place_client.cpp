#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_planning_msgs/PlanPlaceAction.h>

//tf
#include <tf/transform_datatypes.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_plan_place");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPlaceAction> ac("youbot_3/motion_planning/plan_place", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  motion_planning_msgs::PlanPlaceGoal goal;
  goal.place_object = true;


  ac.sendGoal(goal);

  //exit
  return 0;
}
