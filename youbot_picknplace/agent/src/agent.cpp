#include <ros/ros.h>
// actionlib
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
// messages
#include <motion_planning_msgs/PlanObjectDetectionAction.h>
#include <motion_planning_msgs/PlanPlaceAction.h>
#include <motion_planning_msgs/PlanPickAction.h>
#include <motion_planning_msgs/PlanGoHomeAction.h>
#include <motion_planning_msgs/PlanListenAoiAction.h>
#include <motion_planning_msgs/PlanApproachObjectAction.h>
#include <motion_planning_msgs/PlanSensedApproachAction.h>
#include "nav_msgs/Odometry.h"

geometry_msgs::Point base_position;
void currPositionCallback(const nav_msgs::Odometry::ConstPtr& msg);

int main (int argc, char **argv) {
  ros::init(argc, argv, "agent");

  ros::Time::init();
  ros::NodeHandle nh("~");

  if (argc != 2) {
    ROS_INFO("Please provide one argument: 1=motion 0=no motion");
    return 0;
  }

  bool going = true;
  bool success = false;
  ros::Rate r(10);
  // approaching state
  bool approaching_object = false;
  bool object_detected_far = false;


  // states:
  // 0 listen + slightly approach area of interest
  // 1 sensed approach
  // 2 object detection
  // 3 pick
  // 4 place
  // 5 end
  int state = 0;
  int endstate = 5;
  int jumpstart = 2;

  bool grasp_simulation = false;
  if (atoi(argv[1]) == 0) {
    state = jumpstart;
    grasp_simulation = true;
  }
  bool fake_aoi = grasp_simulation;

  //  states:
  // 0 check front
  // 1 check right
  // 2 check left
  int detection_position = 0;

  char *s = std::getenv("ROBOT_NAME");
  // action lib clients
  actionlib::SimpleActionClient<motion_planning_msgs::PlanListenAoiAction> aoi_ac(std::string(s) + "/motion_planning/plan_listen_aoi", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanObjectDetectionAction> obj_ac(std::string(s) + "/motion_planning/plan_detection", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPickAction> pick_ac(std::string(s) + "/motion_planning/plan_pick", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanPlaceAction> place_ac(std::string(s) + "/motion_planning/plan_place", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanGoHomeAction> home_ac(std::string(s) + "/motion_planning/plan_go_home", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanApproachObjectAction> approach_ac(std::string(s) + "/motion_planning/plan_approach_object", true);
  actionlib::SimpleActionClient<motion_planning_msgs::PlanSensedApproachAction> sensed_ac(std::string(s) + "/motion_planning/plan_sensed_approach", true);

  geometry_msgs::PoseStamped object_position_;
  geometry_msgs::Point aoi_position_;

  ros::Subscriber odom_sub = nh.subscribe("/" + std::string(std::getenv("ROBOT_NAME")) + "/odom", 1000, currPositionCallback);

  // START

  while (going) {


    if (approaching_object) {
      if (approach_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        detection_position = approach_ac.getResult()->object_direction;
        ROS_INFO("Base position (%f,%f)", base_position.x, base_position.y);

        state = jumpstart;
        fake_aoi = true;
        approaching_object = false;
      } else if (approach_ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        going = false;
        approaching_object = false;
      }
    }
    // NAVIGATION by AREA OF INTEREST LISTENING
    else if (state == 0) {
      ROS_INFO("Waiting for area of interest action server to start.");
      // wait for the action server to start
      aoi_ac.waitForServer(); //will wait for infinite time
      ROS_INFO("Area of interest Action server started, sending goal.");
      // send a goal to the action
      motion_planning_msgs::PlanListenAoiGoal nav_goal;
      nav_goal.find = true;
      aoi_ac.sendGoal(nav_goal);
      state = 1;

    } else if (state == 1) {
      if (aoi_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Waiting for Sensed Approach action server to start.");
        // wait for the action server to start
        sensed_ac.waitForServer(); //will wait for infinite time
        ROS_INFO("Sensed approach Action server started, sending goal.");
        // send a goal to the action
        motion_planning_msgs::PlanSensedApproachGoal sensed_goal;
        aoi_position_ = aoi_ac.getResult()->aoi_position;
        ROS_INFO("Base position (%f,%f)", base_position.x, base_position.y);

        sensed_goal.aoi_position = aoi_ac.getResult()->aoi_position;
        sensed_ac.sendGoal(sensed_goal);
        state = 2;
      } else if (aoi_ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        going = false;
      }
    } else if (state == 2) {
      // TODO: improve logic here
      //  issue being that state 1 may be the initial state and aoi_ac may not have action running
      if (fake_aoi) {
        obj_ac.waitForServer();
        ROS_INFO("Detect Action server started, sending goal. Starting at position %d", detection_position);
        // send a goal to the action
        motion_planning_msgs::PlanObjectDetectionGoal obj_goal;
        obj_goal.detect = detection_position;
        obj_ac.sendGoal(obj_goal);
        state = 3;
      } else if (sensed_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Base position (%f,%f)", base_position.x, base_position.y);
        object_detected_far = true;
        object_position_ = sensed_ac.getResult()->pose;
        ROS_INFO("Object sensed at (%f,%f).", object_position_.pose.position.x, object_position_.pose.position.y);
        state = 3;
      } else if (sensed_ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("Detect Action server started, sending goal.");
        // if failed start default object detection procedure
        motion_planning_msgs::PlanObjectDetectionGoal obj_goal;
        obj_goal.detect = 0;
        obj_ac.sendGoal(obj_goal);
        state = 3;
      }
    } else if (state == 3) {
      if (object_detected_far) {
        ROS_INFO("Object sensed from far");

        object_detected_far = false;
        detection_position = 0;
        // PICK
        ROS_INFO("Waiting for Pick action server to start.");
        // wait for the action server to start
        pick_ac.waitForServer(); //will wait for infinite time
        ROS_INFO("Pick Action server started, sending goal.");
        // send a goal to the action
        motion_planning_msgs::PlanPickGoal pick_goal;
        pick_goal.object_pose = object_position_;
        ROS_INFO("Object to pick is at (%f,%f).", pick_goal.object_pose.pose.position.x, pick_goal.object_pose.pose.position.y);
        pick_ac.sendGoal(pick_goal);
        state = 4;
      } else if (obj_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        detection_position = obj_ac.getResult()->state;
        // PICK
        ROS_INFO("Waiting for Pick action server to start.");
        // wait for the action server to start
        pick_ac.waitForServer(); //will wait for infinite time
        ROS_INFO("Pick Action server started, sending goal.");
        // send a goal to the action
        motion_planning_msgs::PlanPickGoal pick_goal;
        object_position_ = obj_ac.getResult()->pose;
        pick_goal.object_pose = object_position_;
        ROS_INFO("Object to pick is at (%f,%f).", pick_goal.object_pose.pose.position.x, pick_goal.object_pose.pose.position.y);
        pick_ac.sendGoal(pick_goal);
        state = 4;
      } else if (obj_ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        going = false;
      }
    } else if (state == 4) {
      // TODO fix case when out of reach
      if (pick_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        // PLACE
        ROS_INFO("Waiting for Place server to start.");
        // wait for the action server to start
        place_ac.waitForServer(); //will wait for infinite time
        ROS_INFO("Place Action server started, sending goal.");
        // send a goal to the action
        motion_planning_msgs::PlanPlaceGoal place_goal;
        place_goal.place_object = true;

        place_ac.sendGoal(place_goal);
        state = 5;
      } else if (pick_ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        if (pick_ac.getResult()->success == -2) {
          ROS_WARN("Out of reach.");
          // APPROACH NEARBY (OUT OF REACH) OBJECT
          ROS_INFO("Approaching Object Action, sending goal.");
          // send a goal to the action
          motion_planning_msgs::PlanApproachObjectGoal approach_goal;

          approach_goal.fake_aoi = grasp_simulation;
          if (!grasp_simulation) {
            // use a relative aoi to current position
            geometry_msgs::Point relative_aoi = aoi_position_;
            relative_aoi.x -= base_position.x;
            relative_aoi.y -= base_position.y;
            approach_goal.aoi_position = relative_aoi;
            ROS_INFO("Base position (%f,%f)", base_position.x, base_position.y);
            ROS_INFO("Relative AOI is (%f,%f)", relative_aoi.x, relative_aoi.y);
          }
          approach_goal.object_position = object_position_.pose.position;

          approach_ac.sendGoal(approach_goal);
          approaching_object = true;
        } else
          going = false;
      }
    }  else if (state == endstate) {
      if (place_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        success = true;
        going = false;
      } else if (place_ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        success = false;
        going = false;
      }
    }

    ros::spinOnce();
    r.sleep();
  }// end while going

  if (success) {
    ROS_INFO("Task successful!");
  } else {
    ROS_INFO("Task failed! At state: %d", state);
  }

  // DEFAULT BEHAVIOUR AT END OF EXECUTION
  // GO HOME
  ROS_INFO("Waiting for Go home server to start.");
  // wait for the action server to start
  home_ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Go Home Action server started, sending goal.");
  ROS_INFO("Goint to home pose");
  // send a goal to the action
  motion_planning_msgs::PlanGoHomeGoal home_goal;
  home_goal.go_home = true;

  home_ac.sendGoal(home_goal);

  going = true;
  while (going) {
    if (home_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Done!");
      going = false;
    } else if (home_ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      ROS_INFO("Failed going to home positon");
      going = false;
    }
    ros::spinOnce();
    r.sleep();
  }

  //exit
  return 0;
}

void currPositionCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // const nav_msgs::Odometryt& odom = msg;
  const geometry_msgs::Point position = msg->pose.pose.position;
  base_position = position;
}
