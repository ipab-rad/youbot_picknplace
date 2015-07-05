#include <ros/ros.h>
// actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// messages
#include <motion_planning_msgs/PlanObjectDetectionAction.h>
#include <motion_planning_msgs/PlanPlaceAction.h>
#include <motion_planning_msgs/PlanPickAction.h>
//tf
#include <tf/transform_datatypes.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "agent");

// OBJECT DETECTION
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<motion_planning_msgs::PlanObjectDetectionAction> obj_ac("motion_planning/plan_detection", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  obj_ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  motion_planning_msgs::PlanObjectDetectionGoal obj_goal;
  obj_goal.detect = true;

  obj_ac.sendGoal(obj_goal);



// PICK
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPickAction> pick_ac("motion_planning/plan_pick", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  pick_ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  motion_planning_msgs::PlanPickGoal pick_goal;
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "base_footprint";
  geometry_msgs::Quaternion quat;
  quat = tf::createQuaternionMsgFromRollPitchYaw(-3.129, 0.0549, 1.686);
  target_pose.pose.orientation.x = quat.x;
  target_pose.pose.orientation.y = quat.y;
  target_pose.pose.orientation.z = quat.z;
  target_pose.pose.orientation.w = quat.w;
  ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
  target_pose.pose.position.x = 0.12;
  target_pose.pose.position.y = -0.25;
  target_pose.pose.position.z = 0.0;
  pick_goal.object_pose = target_pose;


  pick_ac.sendGoal(pick_goal);




// PLACE
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPlaceAction> place_ac("motion_planning/plan_place", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  place_ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  motion_planning_msgs::PlanPlaceGoal place_goal;
  place_goal.place_object = true;


  place_ac.sendGoal(place_goal);




  //exit
  return 0;
}
