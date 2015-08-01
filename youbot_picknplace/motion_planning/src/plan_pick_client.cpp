#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_planning_msgs/PlanPickAction.h>
#include <motion_msgs/GripperPose.h>

//tf
#include <tf/transform_datatypes.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "test_plan_pick");
  ros::NodeHandle n;

  if ( argc != 2 && argc != 4)
    return 0;


  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPickAction> ac("youbot_1/motion_planning/plan_pick", true);
  ros::ServiceClient pose_c = n.serviceClient<motion_msgs::GripperPose>("youbot_1/motion/gripper_pose");

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  motion_planning_msgs::PlanPickGoal goal;
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id = "youbot_1/base_footprint";

  if (argc == 2) {
    int command = atoi(argv[1]);
    // simulate some cube poses wrt to base frame
    if (command == 1) {
      // face forward
      target_pose.pose.position.x = 0.42;
      target_pose.pose.position.y = -0.19;
      target_pose.pose.position.z = 0.02;
      target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.41, 0.048, -1.97);
    } else if (command == 2) {
      // face left
      target_pose.pose.position.x = 0.09;
      target_pose.pose.position.y = 0.29;
      target_pose.pose.position.z = 0.02;
      target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.01, -0.002, -0.07);
    } else if (command == 3) {
      // face right
      target_pose.pose.position.x = 0.11;
      target_pose.pose.position.y = -0.3;
      target_pose.pose.position.z = 0.02;
      target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.01, -0.01, 3.06);
    } else if (command == 4) {
      // face right for controlled simulation
      target_pose.pose.position.x = 0.12;
      target_pose.pose.position.y = -0.29;
      target_pose.pose.position.z = 0.0;
      target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.01, -0.01, 3.12);
    } else if (command == 5) {
      // face forward
      target_pose.pose.position.x = 0.44;
      target_pose.pose.position.y = -0.001;
      target_pose.pose.position.z = 0.001;
      target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.41, 0.048, -1.97);
    }
  } else {
    target_pose.pose.position.x = atof(argv[1]);
    target_pose.pose.position.y = atof(argv[2]);
    target_pose.pose.position.z = atof(argv[3]);
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-0.41, 0.048, -1.97);
  }
  goal.object_pose = target_pose;
  ac.sendGoal(goal);

  ac.waitForResult();
  ROS_INFO("Action server result: %d", ac.getResult()->success);


  //exit
  return 0;
}
