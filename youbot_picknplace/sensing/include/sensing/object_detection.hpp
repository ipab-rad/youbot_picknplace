#ifndef OBJECT_DETECTION_SERVER_HPP
#define OBJECT_DETECTION_SERVER_HPP

// ROS
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

// Messages
#include <sensing_msgs/DetectObjectAction.h>
#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "object_recognition_msgs/RecognizedObject.h"
// tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class DetectObjectAction {
 protected:
  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line.
  // Otherwise strange error may occur.
  actionlib::SimpleActionServer<sensing_msgs::DetectObjectAction> as_;
  std::string action_name_;
  sensing_msgs::DetectObjectFeedback feedback_;
  sensing_msgs::DetectObjectResult result_;

 public:
  DetectObjectAction(ros::NodeHandle nh, std::string name);

  ~DetectObjectAction(void);

  void init();

  void goalCB();

  void preemptCB();

  void executeCB();

  void detectedCB(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);

 private:
  geometry_msgs::PoseStamped object_pose_;

  // object detection
  ros::Subscriber object_sub_ ;
  bool object_found_;

};


#endif /* OBJECT_DETECTION_SERVER_HPP */
