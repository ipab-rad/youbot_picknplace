#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_msgs/MoveToPostureAction.h>

//tf
#include <tf/transform_datatypes.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_movetoposture");

  if (argc != 2) {
    ROS_INFO("Usage: argument1=desired posture");
    return 0;
  }
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<motion_msgs::MoveToPostureAction> ac("youbot_3/motion/move_to_posture", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  motion_msgs::MoveToPostureGoal goal;
  goal.posture = argv[1];


  ac.sendGoal(goal);

  //exit
  return 0;
}
