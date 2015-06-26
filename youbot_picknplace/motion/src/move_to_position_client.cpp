#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_msgs/MoveToPositionAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_movetoposition");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<motion_msgs::MoveToPositionAction> ac("motion/move_to_position", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  motion_msgs::MoveToPositionGoal goal;
  goal.position.x = 0.12;
  goal.position.y = -0.15;
  goal.position.z = 0.21;

  ac.sendGoal(goal);

  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action state: %s",state.toString().c_str());

  //exit
  return 0;
}