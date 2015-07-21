#include "navigation/move_to_position.hpp"

MoveToPositionAction::MoveToPositionAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  nav_sub_(nh.subscribe("/odom", 1000, &MoveToPositionAction::currPositionCallback,this)),
  nav_pub_(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000)) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&MoveToPositionAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&MoveToPositionAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting MoveToPosition server");
  as_.start();
}

MoveToPositionAction::~MoveToPositionAction(void) {
}

void MoveToPositionAction::init() {
  odom_received_ = false;
  distance_tol_ = 0.05; // 5cm
  std_vel_ = 0.1;
}

void MoveToPositionAction::goalCB() {
  boost::shared_ptr< const navigation_msgs::MoveToPositionGoal > goal = as_.acceptNewGoal();
  relative_= goal->relative;
  if(!relative_)
    target_position_ = goal->position;
  else{
    target_position_.x = goal->position.x + curr_position_.x;
    target_position_.y = goal->position.y + curr_position_.y;
    target_position_.z = goal->position.z + curr_position_.z;
  }
  this->executeCB();
}

void MoveToPositionAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}

void MoveToPositionAction::executeCB() {
  bool going = true;
  bool success = false;
  timed_out_ = false;

  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());
  feedback_.curr_state = 0;
  as_.publishFeedback(feedback_);

  // start timer
  timer_ = nh_.createTimer(ros::Duration(30), &MoveToPositionAction::timerCB, this, true);

  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    // TODO
    // compute direction of movement needed
    // set nav twist msg
    setTargetTwist(target_position_.x - curr_position_.x, target_position_.y - curr_position_.y, false);

    // send twist message
    nav_pub_.publish(nav_pub_msg_);


    // monitor our position
    // if close enough then stop
    distance_ = sqrt(pow(curr_position_.x - target_position_.x, 2) +
                   pow(curr_position_.y - target_position_.y, 2));

    // ROS_INFO("TARGET POSITION: x:%f y:%f", target_position_.x, target_position_.y);
    // ROS_INFO("CURRENT POSITION: x:%f y:%f", curr_position_.x, curr_position_.y);
    // ROS_INFO("Current distance to desired position: %f", distance_);

    if(distance_<distance_tol_)
    {
      going = false;
      success = true;
    }

    // if timeout then stop
    if (timed_out_) {
      ROS_INFO("%s: Timed out", action_name_.c_str());
      // TODO: set as preempted?
      going = false;
    }

    ros::spinOnce();
    r.sleep();
  }

  // set target stop twist msg
  setTargetTwist(0.0, 0.0, true);
  nav_pub_.publish(nav_pub_msg_);

  ROS_INFO("TARGET POSITION: x:%f y:%f", target_position_.x, target_position_.y);
  ROS_INFO("CURRENT POSITION: x:%f y:%f", curr_position_.x, curr_position_.y);
  ROS_INFO("Current distance to desired position: %f", distance_);
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
  timer_.stop();
}

void MoveToPositionAction::timerCB(const ros::TimerEvent& event) {
  timed_out_ = true;
}

void MoveToPositionAction::setTargetTwist(double x, double y, bool stop){
  geometry_msgs::Twist msg;
  double ratio;
  if(stop){
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
  }else if(fabs(x)>fabs(y)){
    ratio = fabs(y/x);
    if(x>0)
      msg.linear.x = std_vel_;
    else
      msg.linear.x = -std_vel_;
    if(y>0)
      msg.linear.y = ratio*std_vel_;
    else
      msg.linear.y = -ratio*std_vel_;      
  }else{
    ratio = fabs(x/y);
    if(x>0)
      msg.linear.x = ratio*std_vel_;
    else
      msg.linear.x = -ratio*std_vel_;
    if(y>0)
      msg.linear.y = std_vel_;
    else
      msg.linear.y = -std_vel_;
  }
  ROS_INFO("Twist set to: x:%f y:%f",msg.linear.x,msg.linear.y);
  nav_pub_msg_ = msg;
}

void MoveToPositionAction::currPositionCallback(const nav_msgs::Odometry::ConstPtr& msg){
  // const nav_msgs::Odometryt& odom = msg;
  const geometry_msgs::Point position = msg->pose.pose.position;
  curr_position_ = position;
  odom_received_ = true;
}