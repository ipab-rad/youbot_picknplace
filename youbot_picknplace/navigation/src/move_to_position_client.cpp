#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <navigation_msgs/MoveToPositionAction.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_movetoposition");

  if (argc!=3)
    return 0;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<navigation_msgs::MoveToPositionAction> ac("navigation/move_to_position", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  navigation_msgs::MoveToPositionGoal goal;
  geometry_msgs::Point target_position;

  target_position.x = atof(argv[1]);
  target_position.y = atof(argv[2]);
  target_position.z = 0.0;
  goal.position = target_position;

  ac.sendGoal(goal);

  //exit
  return 0;
}
