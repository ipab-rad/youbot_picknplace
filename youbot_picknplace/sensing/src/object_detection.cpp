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
  // set timer
  // TODO: adjust the timer based on input
  timed_out_ = false;
  timer_ = nh_.createTimer(ros::Duration(20), &DetectObjectAction::timerCB, this, true);


  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // TODO turn off actual obj detection
    if (object_found_) {
      //  TODO: send the new action (approach)
      ROS_INFO("Sent approach plan action message (NOT)");

      going = false;
      success = true;
    }

    if (timed_out_) {
      ROS_INFO("%s: Timed out", action_name_.c_str());
      // TODO: set as preempted?
      going = false;
    }

    ros::spinOnce();
    r.sleep();
  }

  feedback_.curr_state = 3;
  as_.publishFeedback(feedback_);

  if (success) {
    result_.success = true;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = false;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }
}

void DetectObjectAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}

// void DetectObjectAction::detectedCB(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg) {
void DetectObjectAction::detectedCB(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg) {
  tf::StampedTransform stransform;
  tf::TransformListener listener;
  ROS_INFO("Object message received");
  // for (size_t i_msg = 0; i_msg < msg->objects.size(); ++i_msg) {
  //   // TODO ensure this is running correctly with indices ++i vs i++ in loop
  //   // const object_recognition_msgs::RecognizedObject& object = msg->objects[i_msg];
  //   const geometry_msgs::Pose obj_pose = object.pose.pose.pose;

  //   ROS_INFO("Object was recognized. Key: %s", object.type.key.c_str());
  //   try {
  //     ROS_INFO("Frame ID: %s", object.pose.header.frame_id.c_str());
  //     listener.waitForTransform("/base_footprint", object.pose.header.frame_id.c_str(), ros::Time(0), ros::Duration(13.0) );

  //     geometry_msgs::PoseStamped pin;
  //     pin.header = object.pose.header;
  //     pin.pose = obj_pose;
  //     geometry_msgs::PoseStamped pout;
  //     listener.transformPose("/base_footprint", pin, pout);

  //     // DEBUG
  //     // listener.lookupTransform("/base_footprint", object.pose.header.frame_id.c_str(), ros::Time(0), stransform);
  //     // ROS_INFO("Computed transform to /base_footprint, Point (x,y,z): (%f,%f,%f)", stransform.getOrigin().x(), stransform.getOrigin().y(), stransform.getOrigin().z());
  //     ROS_INFO("3D point in frame of /base_footprint, Point (x,y,z): (%f,%f,%f)", pout.pose.position.x, -pout.pose.position.y, pout.pose.position.z);

  //     // TODO: send geometry_msgs::PoseStamped pout to plan approach
  //     // ATTENTION: it seems coordinates z-y are flipped, so modify it
  //   } catch (tf::TransformException ex) {
  //     ROS_ERROR("%s", ex.what());
  //     ros::Duration(1.0).sleep();
  //   }

  object_found_ = true;
  //   b
  // }
}
