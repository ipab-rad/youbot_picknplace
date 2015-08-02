#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_planning_msgs/PlanApproachObjectAction.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_plan_approach_object");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<motion_planning_msgs::PlanApproachObjectAction> ac("youbot_3/motion_planning/plan_approach_object", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  motion_planning_msgs::PlanApproachObjectGoal goal;
  geometry_msgs::Point position;
  position.x = 0.05;
  position.y = 0.1;
  goal.position = position;
  ac.sendGoal(goal);

  //exit
  return 0;
}
