/**
 * @file      agent_ros_nav.cpp
 * @brief     Main exporation agent node
 * @author    Alexandre Silva <s1464657@sms.ed.ac.uk>
 * @date      2015-08-20
 */

#include <ros/ros.h>
// actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// messages
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
// tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <string>
#include <sstream>

using namespace std;

nav_msgs::OccupancyGrid costmap_;
bool received_costmap_ = false;

// stops youbot
// publishes 10 msgs over a 1s period
void stopYoubot(ros::Publisher pub) {
  // stop msg Twist
  geometry_msgs::Twist stop_msg;
  stop_msg.linear.x = 0.0;
  stop_msg.linear.y = 0.0;
  stop_msg.linear.z = 0.0;
  stop_msg.angular.x = 0.0;
  stop_msg.angular.y = 0.0;
  stop_msg.angular.z = 0.0;

  ros::Rate r(10);

  for (int i = 0; i < 10; i++) {
    pub.publish(stop_msg);
    ros::spinOnce();
    r.sleep();
  }
}

// processes costmap to find area(s) of interest
// identifies maxima that have surrounding free space
std::vector<geometry_msgs::Point> processMap() {
  std::vector<geometry_msgs::Point> AOIs;
  size_t size = costmap_.data.size();

  ROS_INFO("Cell count of costmap: %lu", size);
  ROS_INFO("Width %d", costmap_.info.width);
  ROS_INFO("Height %d", costmap_.info.height);

  int width = costmap_.info.width;
  int height = costmap_.info.height;
  double resolution = costmap_.info.resolution;
  // geometry_msgs::Pose map_pose = costmap_.info.origin;

  int index = 0;
  int offset = 20;
  int thresh = 0; // costmap value
  bool local_max = false;

  // last accepted Area of Interest
  int last_i = -1;
  int last_j = -1;
  double sameAoiThresh = 10.0; // cells
  bool sameAoi = false;

  // finding local maxima points
  for (int i = offset; i < (width - offset); i++) {
    for (int j = offset; j < (height - offset); j++) {
      index = width * i + j;

      if (costmap_.data[index] == 100) {
        local_max = costmap_.data[width * (i + offset) + j] == thresh && costmap_.data[width * (i - offset) + j] == thresh
                    && costmap_.data[width * i + j + offset] == thresh && costmap_.data[width * i + j - offset] == thresh;
        sameAoi = sqrt(pow(i - last_i, 2) + pow(j - last_j, 2)) < sameAoiThresh;
        if (local_max && !sameAoi) {
          ROS_INFO("Found AOI at (%d,%d)", i, j);
          last_i = i;
          last_j = j;
          geometry_msgs::Point aoi;
          aoi.x = (j - width / 2) * resolution;
          aoi.y = (i - width / 2) * resolution;
          aoi.z = 0.0;
          AOIs.push_back(aoi);
          ROS_INFO("AOI info:");
          ROS_INFO("X offset: %f", (j - width / 2)*resolution);
          ROS_INFO("Y offset: %f", (i - width / 2)*resolution);
        }
      }
    }
  }
  ROS_INFO("costmap local has %lu AOIs", AOIs.size());

  for (size_t i = 0; i < AOIs.size(); i++) {
    // find transformation to manipulating youbot for each AOI
    try {
      tf::StampedTransform stransform;
      tf::TransformListener listener;
      geometry_msgs::PoseStamped pin;
      pin.header.frame_id = "/youbot_4/base_footprint";
      pin.header.stamp = ros::Time(0);
      geometry_msgs::Quaternion orientation;
      orientation.x = 0.0;
      orientation.y = 0.0;
      orientation.z = 0.0;
      orientation.w = 1.0;
      pin.pose.orientation = orientation;
      pin.pose.position.x = AOIs[i].x;
      pin.pose.position.y = AOIs[i].y;
      geometry_msgs::PoseStamped pout;

      listener.waitForTransform("/youbot_3/base_footprint", pin.header.frame_id.c_str(), ros::Time(0), ros::Duration(13.0) );
      listener.transformPose("/youbot_3/base_footprint", pin, pout);
      // ROS_INFO("AOI in frame of Youbot 3, Point (x,y,z): (%f,%f,%f)", pout.pose.position.x, pout.pose.position.y, pout.pose.position.z);
      AOIs[i].x = pout.pose.position.x;
      AOIs[i].y = pout.pose.position.y;

    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  return AOIs;
}

// stores a copy of the costmap
void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  // ROS_INFO("Costmap msg received");
  costmap_ = (*msg);
  received_costmap_ = true;
}

size_t promptUser(  std::vector<geometry_msgs::Point> aois) {

  for (size_t i = 0; i < aois.size(); i++) {
    ROS_INFO("AOI %lu: (%f,%f,0.0)", i, aois[i].x, aois[i].y);
  }
  ROS_INFO("Which area would you like to explore?");

  string input = "";
  size_t myNumber = 0;
  while (true) {
    cout << "Please enter a valid AOI index: ";
    getline(cin, input);

    // This code converts from string to number safely.
    stringstream myStream(input);
    if (myStream >> myNumber && myNumber < aois.size() && myNumber >= 0) {
      break;
    }
    cout << "Invalid AOI, please try again" << endl;
  }
  return myNumber;
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "agent_nav");
  ros::NodeHandle n;


  if (argc != 2)
    return 0;
  bool going = true;
  bool success = false;
  // moving state
  bool moving = false;

  // states:
  // 0 move forward
  // 1 move left
  // 2 move back
  // 3 move right
  // 4 end
  int state = 0;
  int endstate = 4;
  if (atoi(argv[1]) == 0)
    state = 4;

  // action lib client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_ac("youbot_4/move_base", true);
  // publisher to stop youbot
  ros::Publisher nav_pub_ = n.advertise<geometry_msgs::Twist>("youbot_4/cmd_vel", 1000);
  ros::Subscriber costm_sub_ = n.subscribe("/youbot_4/move_base_node/local_costmap/costmap", 1, &costmapCB);
  ros::Publisher aoi_pub = n.advertise<geometry_msgs::Point>("areas_of_interest", 10);



  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  move_ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Running.");

  // move base goal
  move_base_msgs::MoveBaseGoal mov_goal;
  geometry_msgs::Quaternion base_orientation;
  base_orientation.x = 0.0;
  base_orientation.y = 0.0;
  base_orientation.z = 0.0;
  base_orientation.w = 1.0;

  double movement_size = 2.0; //meters

  ros::Rate r(10);

  // START
  while (ros::ok() && going) {

    // NAVIGATION
    if (moving) {
      if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        stopYoubot(nav_pub_);
        state++;
        received_costmap_ = false;
        moving = false;
      } else if (move_ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        stopYoubot(nav_pub_);
        going = false;
        moving = false;
      }

    } else if (state == 0) {
      // go forward 1m
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "youbot_4/base_footprint";
      pose.pose.position.x = movement_size;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = base_orientation;
      mov_goal.target_pose = pose;
      move_ac.sendGoal(mov_goal);
      moving = true;

    } else if (state == 1) {
      // move left 1m
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "youbot_4/base_footprint";
      pose.pose.position.x = 0.0;
      pose.pose.position.y = movement_size;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = base_orientation;
      mov_goal.target_pose = pose;
      move_ac.sendGoal(mov_goal);
      moving = true;

    } else if (state == 2) {
      // move back 1m
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "youbot_4/base_footprint";
      pose.pose.position.x = -movement_size;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = base_orientation;
      mov_goal.target_pose = pose;
      move_ac.sendGoal(mov_goal);
      moving = true;

    } else if (state == 3) {
      // move back 1m
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "youbot_4/base_footprint";
      pose.pose.position.x = 0.0;
      pose.pose.position.y = -movement_size;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = base_orientation;
      mov_goal.target_pose = pose;
      move_ac.sendGoal(mov_goal);
      moving = true;

    } else if (state == endstate) {
      if (received_costmap_) {
        success = true;
        going = false;
      }
    }
    r.sleep();
    ros::spinOnce();
  }// end while going
  stopYoubot(nav_pub_);



  // processing costmap
  std::vector<geometry_msgs::Point> AOIs = processMap();

  if (AOIs.size() > 0) {
    // prompt use which AOI to choose from
    size_t aoi_index = promptUser(AOIs);

    // publish AOI
    ROS_INFO("AOI %lu chosen", aoi_index);
    aoi_pub.publish(AOIs[aoi_index]);
  }

  if (success) {
    ROS_INFO("Task successful!");
  } else {
    ROS_INFO("Task failed! At state: %d", state);
  }

  // just precaution
  stopYoubot(nav_pub_);
  //exit
  return 0;
}
