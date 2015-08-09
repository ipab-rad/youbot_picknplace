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
// openni
#include "openni2_camera/openni2_device_manager.h"
#include "openni2_camera/openni2_device.h"

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

  bool validateObject(geometry_msgs::Point point);

  void timerCB(const ros::TimerEvent& event);


 private:
  // timer for action timeout
  ros::Timer timer_;
  bool timed_out_;
  int detection_time_;

  geometry_msgs::PoseStamped object_pose_;
  bool detect_;

  // object detection
  // tf::StampedTransform stransform_;
  tf::TransformListener listener_;


  bool object_found_;
  bool object_validated_;
  int validation_count_;
  int required_validations_;
  double sim_threshold_;

  // youbot name
  std::string youbot_;

};

bool checkSimilarity(double p1, double p2, double threshold);


#endif /* OBJECT_DETECTION_SERVER_HPP */
