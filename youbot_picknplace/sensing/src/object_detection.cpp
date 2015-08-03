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
  detect_ = false;
  required_validations_ = 2;
  sim_threshold_ = 0.01;
  youbot_ = std::string(std::getenv("ROBOT_NAME"));
}

void DetectObjectAction::goalCB() {
  detect_ = as_.acceptNewGoal()->detect;
  this->executeCB();
}

void DetectObjectAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void DetectObjectAction::executeCB() {
  bool going = detect_;
  bool success = false;
  object_found_ = false;
  object_validated_ = false;
  validation_count_ = 0;

  ros::Rate r(20);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if (object_found_ && object_validated_) {
      going = false;
      success = true;
    }

    ros::spinOnce();
    r.sleep();
  }

  feedback_.curr_state = 3;
  as_.publishFeedback(feedback_);

  if (success) {
    try {
      object_pose_.header.stamp = ros::Time(0);
      geometry_msgs::PoseStamped pout;
      listener_.waitForTransform(youbot_ + "/base_footprint", object_pose_.header.frame_id.c_str(), ros::Time(0), ros::Duration(13.0) );
      listener_.transformPose(youbot_ + "/base_footprint", object_pose_, pout);
      ROS_INFO("Object position wrt to frame /base_footprint, Point (x,y,z): (%f,%f,%f)", pout.pose.position.x, pout.pose.position.y, pout.pose.position.z);
      result_.pose = pout;

    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }

}

void DetectObjectAction::detectedCB(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg) {
  // ROS_INFO("Object message received");

  if (msg->objects.size() == 1 && detect_) {
    const object_recognition_msgs::RecognizedObject& object = msg->objects[0];
    const geometry_msgs::Pose obj_pose = object.pose.pose.pose;

    ROS_INFO("Object was detected. With confidence: %f", object.confidence);
    if (object.confidence > 0.95) {
      object_pose_.header.frame_id = youbot_ + "/" + object.pose.header.frame_id;
      object_pose_.header.stamp = ros::Time(0);
      object_pose_.pose = obj_pose;
      object_found_ = true;

      if (validateObject(obj_pose.position)) {
        detect_ = false;
        object_validated_ = true;
      }
    }
  } else {
    ROS_INFO("No objects detected.");
  }
}

// functions to check if this object has been validated given previous observations
// if an object does not pass the similarity test it will reset the validations counter
// if there have been 'required_validations_' validations it returns true
// false otherwise
bool DetectObjectAction::validateObject(geometry_msgs::Point point) {
  if (!checkSimilarity(point.x, object_pose_.pose.position.x, sim_threshold_)
      || !checkSimilarity(point.y, object_pose_.pose.position.y, sim_threshold_)
      || !checkSimilarity(point.z, object_pose_.pose.position.z, sim_threshold_)) {
    validation_count_ = 0;
    return false;
  }
  validation_count_++;
  ROS_INFO("Similar object position has been received. Validation count at: %d", validation_count_);

  if (validation_count_ == required_validations_)
    return true;
  else
    return false;
}

// returns true if it satisfies the similarity criteria
// false otherwise
bool checkSimilarity(double p1, double p2, double threshold) {
  if (fabs(p1 - p2) < threshold)
    return true;
  else
    return false;
}
