#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sensing_msgs/DetectObjectAction.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_object_detection");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<sensing_msgs::DetectObjectAction> ac("youbot_3/sensing/object_detection", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  sensing_msgs::DetectObjectGoal goal;
  goal.detect = true;


  ac.sendGoal(goal);

  //exit
  return 0;
}
