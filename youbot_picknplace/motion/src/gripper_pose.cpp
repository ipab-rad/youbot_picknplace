#include "motion/gripper_pose.hpp"

GripperPose::GripperPose(ros::NodeHandle nh, std::string name) :
  nh_(nh) {

  // advertise this service
  pose_srv_ =
    nh_.advertiseService("gripper_pose", &GripperPose::compute,this);

  ROS_INFO("Starting GripperPose server");
}

GripperPose::~GripperPose(void) {
}

bool GripperPose::compute(motion_msgs::GripperPose::Request  &req,
         motion_msgs::GripperPose::Response &res) {
  
  double x = req.position.x;
  double y = req.position.y;

  geometry_msgs::Quaternion quat;
  if(x > 0.2){
    // face forward
    quat = tf::createQuaternionMsgFromRollPitchYaw(3.14, 0.0, 0.0);
  }else if(y > 0.0){
    // face left
    quat = tf::createQuaternionMsgFromRollPitchYaw(3.14, 0.0, 1.57);
  }else{
    // face right
    quat = tf::createQuaternionMsgFromRollPitchYaw(3.14, 0.0, -1.57);
  }


  res.orientation = quat;
  ROS_INFO("request: x=%f, y=%f", x, y);
  // ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
