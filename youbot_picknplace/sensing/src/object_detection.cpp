#include "sensing/object_detection.hpp"


DetectObjectAction::DetectObjectAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  object_sub_(nh_.subscribe(
                "/recognized_object_array",
                1,
                &DetectObjectAction::detectedCB, this)) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&DetectObjectAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&DetectObjectAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting DetectObject server");
  as_.start();
}

DetectObjectAction::~DetectObjectAction(void) {
}

void DetectObjectAction::init() {
  object_found_ = false;
}

void DetectObjectAction::goalCB() {
  as_.acceptNewGoal();
  this->executeCB();
}

void DetectObjectAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void DetectObjectAction::executeCB() {
  bool going = true;
  bool success = false;
  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // TODO turn off actual obj detection
    if (object_found_) {
      result_.pose = object_pose_;
      going = false;
      success = true;
    }

    ros::spinOnce();
    r.sleep();
  }

  feedback_.curr_state = 3;
  as_.publishFeedback(feedback_);

  if (success) {
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }
}

// void DetectObjectAction::detectedCB(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg) {
void DetectObjectAction::detectedCB(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg) {
  tf::StampedTransform stransform;
  tf::TransformListener listener;
  ROS_INFO("Object message received");


  if (msg->objects.size() == 1) {
    // const object_recognition_msgs::RecognizedObject& object = msg->objects[0];
    // const geometry_msgs::Pose obj_pose = object.pose.pose.pose;

    // ROS_INFO("Object was recognized. Key: %s", object.type.key.c_str());
    // try {
    //   ROS_INFO("Object detection frame: %s", object.pose.header.frame_id.c_str());
    //   listener.waitForTransform("/base_footprint", object.pose.header.frame_id.c_str(), ros::Time(0), ros::Duration(13.0) );
    //   ROS_INFO("Found transform to robot base");

    //   geometry_msgs::PoseStamped pin;
    //   pin.header = object.pose.header;
    //   pin.pose = obj_pose;
    //   geometry_msgs::PoseStamped pout;
    //   listener.transformPose("/base_footprint", pin, pout);

    //   // DEBUG
    //   // listener.lookupTransform("/base_footprint", object.pose.header.frame_id.c_str(), ros::Time(0), stransform);
    //   // ROS_INFO("Computed transform to /base_footprint, Point (x,y,z): (%f,%f,%f)", stransform.getOrigin().x(), stransform.getOrigin().y(), stransform.getOrigin().z());
    //   ROS_INFO("3D point in frame of /base_footprint, Point (x,y,z): (%f,%f,%f)", pout.pose.position.x, -pout.pose.position.y, pout.pose.position.z);

    //   // TODO
    //   // ATTENTION: it seems coordinates z-y are flipped, so modify it
    //   object_pose_ = pout;

    // } catch (tf::TransformException ex) {
    //   ROS_ERROR("%s", ex.what());
    //   ros::Duration(1.0).sleep();
    // }


    // TEST
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
    object_pose_ = target_pose;
    object_found_ = true;

  }
}
