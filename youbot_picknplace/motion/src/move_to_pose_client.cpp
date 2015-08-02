#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_msgs/MoveToPoseAction.h>

//tf
#include <tf/transform_datatypes.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_movetopose");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<motion_msgs::MoveToPoseAction> ac("youbot_3/motion/move_to_pose", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  motion_msgs::MoveToPoseGoal goal;
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_footprint";
  geometry_msgs::Quaternion quat;
  quat = tf::createQuaternionMsgFromRollPitchYaw(-3.129, 0.0549, 1.686);
  target_pose.pose.orientation.x = quat.x;
  target_pose.pose.orientation.y = quat.y;
  target_pose.pose.orientation.z = quat.z;
  target_pose.pose.orientation.w = quat.w;
  ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
  target_pose.pose.position.x = 0.1;
  target_pose.pose.position.y = -0.3;
  target_pose.pose.position.z = 0.02;
  goal.pose = target_pose;
  goal.distance_tol = 0.01;
  goal.orientation_tol = 0.05;
  goal.planning_time = 20.0;


  ac.sendGoal(goal);

  //exit
  return 0;
}
