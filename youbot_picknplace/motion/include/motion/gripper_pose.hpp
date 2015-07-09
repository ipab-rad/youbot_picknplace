#ifndef GRIPPER_POSE_SERVER_HPP
#define GRIPPER_POSE_SERVER_HPP

// ROS
#include <ros/ros.h>
// Messages
#include <motion_msgs/GripperPose.h>

//tf
#include <tf/transform_datatypes.h>

class GripperPose {
 protected:
  ros::NodeHandle nh_;

 public:
  GripperPose(ros::NodeHandle nh, std::string name);

  ~GripperPose(void);

  bool compute(motion_msgs::GripperPose::Request  &req, motion_msgs::GripperPose::Response &res);


 private:

ros::ServiceServer pose_srv_;

};


#endif /* GRIPPER_POSE_SERVER_HPP */
