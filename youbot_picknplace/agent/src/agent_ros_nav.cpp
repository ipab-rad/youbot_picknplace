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
void processMap() {
  size_t size = costmap_.data.size();

  ROS_INFO("Cell count of costmap: %lu", size);
  ROS_INFO("Width %d", costmap_.info.width);
  ROS_INFO("Height %d", costmap_.info.height);

  int count = 0;
  int width = costmap_.info.width;
  int height = costmap_.info.height;
  double resolution = costmap_.info.resolution;
  // geometry_msgs::Pose map_pose = costmap_.info.origin;

  int index = 0;
  int offset = 20;
  int thresh = 0; // costmap value
  bool local_max = false;
  count = 0;

  // last accepted Area of Interest
  int last_i = -1;
  int last_j = -1;
  double sameAoiThresh = 10.0; // cells
  bool sameAoi = false;
  // best area of interest
  int best_i = -1;
  int best_j = -1;
  double best_dist = 1000.0;

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
          count++;
          ROS_INFO("AOI info:");
          ROS_INFO("X offset: %f", (j - width / 2)*resolution);
          ROS_INFO("Y offset: %f", (i - width / 2)*resolution);
          if (sqrt(pow(i - width / 2, 2) + pow(j - width / 2, 2)) < best_dist) {
            best_i = i;
            best_j = j;
            best_dist = sqrt(pow(i - width / 2, 2) + pow(j - width / 2, 2));
          }
        }
      }
    }
  }
  ROS_INFO("costmap local max count: %d", count);


  // find transformation to manipulating youbot
  if (count > 0) {
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
      pin.pose.position.x = (best_j - width / 2) * resolution;
      pin.pose.position.y = (best_i - width / 2) * resolution;
      geometry_msgs::PoseStamped pout;

      listener.waitForTransform("/youbot_3/base_footprint", pin.header.frame_id.c_str(), ros::Time(0), ros::Duration(13.0) );
      listener.transformPose("/youbot_3/base_footprint", pin, pout);
      ROS_INFO("AOI in frame of Youbot 3, Point (x,y,z): (%f,%f,%f)", pout.pose.position.x, pout.pose.position.y, pout.pose.position.z);

    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }

}

// stores a copy of the costmap
void costmapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  ROS_INFO("Costmap msg received");
  costmap_ = (*msg);
  ROS_INFO("Captured origin: (%f,%f)", costmap_.info.origin.position.x, costmap_.info.origin.position.y);
  ROS_INFO("Captured orientation: (%f,%f,%f,%f)", costmap_.info.origin.orientation.x, costmap_.info.origin.orientation.y, costmap_.info.origin.orientation.z, costmap_.info.origin.orientation.w);

  received_costmap_ = true;
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "agent");
  ros::NodeHandle n;


  if (argc != 2)
    return 0;
  bool going = true;
  bool success = false;

  // states:
  // 0 move forward
  // 1 move left
  // 2 move back
  // 3 move right
  int state = 0;
  if (atoi(argv[1]) == 0)
    state = 4;

  // action lib client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_ac("youbot_4/move_base", true);
  // publisher to stop youbot
  ros::Publisher nav_pub_ = n.advertise<geometry_msgs::Twist>("youbot_4/cmd_vel", 1000);
  ros::Subscriber costm_sub_ = n.subscribe("/youbot_4/move_base_node/local_costmap/costmap", 1, &costmapCB);


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

  double movement_size = 1.5; //meters

  ros::Rate r(10);

  // START
  while (ros::ok() && going) {

    // NAVIGATION
    if (state == 0) {
      // go forward 1m
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "youbot_4/base_footprint";
      pose.pose.position.x = movement_size;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = base_orientation;
      mov_goal.target_pose = pose;

      move_ac.sendGoal(mov_goal);
      move_ac.waitForResult();
      stopYoubot(nav_pub_);

      if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 1;
        received_costmap_ = false;
      } else
        going = false;
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
      move_ac.waitForResult();
      stopYoubot(nav_pub_);

      if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        state = 2;
      else
        going = false;

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
      move_ac.waitForResult();
      stopYoubot(nav_pub_);

      if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        state = 3;
      else
        going = false;
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
      move_ac.waitForResult();
      stopYoubot(nav_pub_);

      if (move_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state = 4;
        received_costmap_ = false;
      } else
        going = false;
    } else if (state == 4) {
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
  processMap();
  // TODO from AOI found, publish to areas of interest topic

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
