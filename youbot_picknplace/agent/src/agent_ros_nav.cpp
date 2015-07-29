#include <ros/ros.h>
// actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// messages
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>

void stopYoubot(ros::Publisher pub){
  // stop msg Twist
  geometry_msgs::Twist stop_msg;
  stop_msg.linear.x = 0.0;
  stop_msg.linear.y = 0.0;
  stop_msg.linear.z = 0.0;
  stop_msg.angular.x = 0.0;
  stop_msg.angular.y = 0.0;
  stop_msg.angular.z = 0.0;

  ros::Rate r(10);

  for(int i=0;i<10;i++){
    pub.publish(stop_msg);
    ros::spinOnce();
    r.sleep();
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "agent");
  ros::NodeHandle n;

  bool going = true;
  bool success = false;

  // states:
  // 0 move forward
  // 0 move left
  // 0 move back
  // 3 move right
  int state = 0;

  // action lib client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_ac("youbot_4/move_base", true);
  // publisher to stop youbot
  ros::Publisher nav_pub_ = n.advertise<geometry_msgs::Twist>("youbot_4/cmd_vel", 1000);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  move_ac.waitForServer(); //will wait for infinite time

  // move base goal
  move_base_msgs::MoveBaseGoal mov_goal;
  geometry_msgs::Quaternion base_orientation;
  base_orientation.x = 0.0;
  base_orientation.y = 0.0;
  base_orientation.z = 0.0;
  base_orientation.w = 1.0;

  // START
  while(ros::ok() && going)
  {

    // NAVIGATION
    if(state==0){
      // go forward 1m
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "youbot_4/base_footprint";
      pose.pose.position.x = 1.0;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = base_orientation;
      mov_goal.target_pose = pose;

      move_ac.sendGoal(mov_goal);
      move_ac.waitForResult();
      stopYoubot(nav_pub_);

      if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        state = 1;
      else
        going = false;
    }
    else if(state==1){
      // move left 1m
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "youbot_4/base_footprint";
      pose.pose.position.x = 0.0;
      pose.pose.position.y = 1.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = base_orientation;

      mov_goal.target_pose = pose;

      move_ac.sendGoal(mov_goal);
      move_ac.waitForResult();
      stopYoubot(nav_pub_);

      if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        state = 2;
      else
        going = false;

    }
    else if(state==2){
      // move back 1m
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "youbot_4/base_footprint";
      pose.pose.position.x = -1.0;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = base_orientation;

      mov_goal.target_pose = pose;

      move_ac.sendGoal(mov_goal);
      move_ac.waitForResult();
      stopYoubot(nav_pub_);

      if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        state = 3;
      else
        going = false;
    }
    else if(state==3){
      // move back 1m
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "youbot_4/base_footprint";
      pose.pose.position.x = 0.0;
      pose.pose.position.y = -1.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = base_orientation;

      mov_goal.target_pose = pose;

      move_ac.sendGoal(mov_goal);
      move_ac.waitForResult();
      stopYoubot(nav_pub_);

      if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        success = true;
      
      going = false;
    }
  }// end while going
  stopYoubot(nav_pub_);

  if (success) {
    ROS_INFO("Task successful!");
  } else {
    ROS_INFO("Task failed! At state: %d", state);
  }

  //exit
  return 0;
}
