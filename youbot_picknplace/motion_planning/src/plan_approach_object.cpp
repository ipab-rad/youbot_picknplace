/**
 * @file      plan_approach_object.cpp
 * @brief     Higher-level action to plan approaching of object given its position and area of interest position
 * @author    Alexandre Silva <s1464657@sms.ed.ac.uk>
 * @date      2015-08-20
 */

#include "motion_planning/plan_approach_object.hpp"

PlanApproachObjectAction::PlanApproachObjectAction(ros::NodeHandle nh, std::string name) :
  nh_(nh),
  as_(nh_, name, false),
  action_name_(name),
  ac_move_("navigation/move_to_position", true) {
  //register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&PlanApproachObjectAction::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&PlanApproachObjectAction::preemptCB, this));

  this->init();
  ROS_INFO("Starting PlanApproachObject server");
  as_.start();
}

PlanApproachObjectAction::~PlanApproachObjectAction(void) {
}

void PlanApproachObjectAction::init() {
  min_grasp_dist_ = 0.3;
}

void PlanApproachObjectAction::goalCB() {
  boost::shared_ptr< const motion_planning_msgs::PlanApproachObjectGoal > goal = as_.acceptNewGoal();
  object_position_ = goal->object_position;
  fake_aoi_ = goal->fake_aoi;
  if (!fake_aoi_) {
    aoi_position_ = goal->aoi_position;
    this->executeCB();
  } else
    this->executeFakeCB();
}

void PlanApproachObjectAction::preemptCB() {
  ROS_INFO("Preempt");
  as_.setPreempted();
}


void PlanApproachObjectAction::executeCB() {
  bool success = false;
  bool going = true;

  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());


  bool moving = false;
  // states
  // 0 move back (x dimension) if necessary to not run over object
  // 1 align y dimension
  // 2 align x dimension
  // 3 end
  int state = 0;
  int endstate = 3;

  setPositionGoals();

  // base moving
  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }

    if (moving) {
      if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        state++;
        moving = false;
      } else if (ac_move_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        // failed
        success = false;
        going = false;
        moving = false;
      }
    } else if (state == 0) {
      if (!skip_1) {
        // move to first position
        ac_move_.waitForServer();
        ROS_INFO("Navigating to object, part 1");
        ac_move_.sendGoal(position_goal);
        moving = true;
      } else {
        state++;
      }
    } else if (state == 1) {
      if (!skip_2) {
        // move to second position
        ac_move_.waitForServer();
        ROS_INFO("Navigating to object, part 2");
        ac_move_.sendGoal(position_goal_2);
        moving = true;
      } else {
        state++;
      }
    } else if (state == 2) {
      if (!skip_3) {
        // move to first position
        ac_move_.waitForServer();
        ROS_INFO("Navigating to object, part 3");
        ac_move_.sendGoal(position_goal_3);
        moving = true;
      } else {
        state++;
      }
    } else if (state == endstate) {
      // success
      success = true;
      going = false;
    }

    ros::spinOnce();
    r.sleep();
  }

  if (success) {
    result_.success = success;
    result_.object_direction = object_direction_;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = success;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }
}

void PlanApproachObjectAction::executeFakeCB() {
  bool success = false;
  bool going = true;

  ros::Rate r(10);
  ROS_INFO("Executing goal for %s", action_name_.c_str());

  target_position_ = computeFakeMovement(min_grasp_dist_, object_position_);
  // message goals
  navigation_msgs::MoveToPositionGoal position_goal_;

  // move to pose action
  ac_move_.waitForServer();
  position_goal_.position = target_position_;
  position_goal_.relative = true;
  ROS_INFO("Navigating to object");
  ac_move_.sendGoal(position_goal_);

  // base moving
  while (going) {
    if (as_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      going = false;
    }


    if (ac_move_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      // success
      success = true;
      going = false;
    } else if (ac_move_.getState() == actionlib::SimpleClientGoalState::ABORTED) {
      // failed
      success = false;
      going = false;
    }

    ros::spinOnce();
    r.sleep();
  }

  if (success) {
    result_.success = success;
    ROS_INFO("%s: Succeeded!", action_name_.c_str());
    as_.setSucceeded(result_);
  } else {
    result_.success = success;
    ROS_INFO("%s: Failed!", action_name_.c_str());
    as_.setAborted(result_);
  }
}

void PlanApproachObjectAction::setPositionGoals() {
  // constants
  const double base_offset = 0.143;
  const double front_range = 0.2;
  const double running_over_x = 0.3;
  const double guarateed_reach = 0.3 + base_offset;
  // evaluates if object is positioned in front of AOI
  double y_diff = aoi_position_.y - object_position_.y;

  skip_1 = false;
  skip_2 = false;
  skip_3 = false;

  // object is in front of AOI
  if (fabs(y_diff) < front_range) {
    ROS_WARN("Object in front of AOI");
    object_direction_ = 0;
    position_goal.position.y = object_position_.y;
    position_goal.position.x = 0.0;
    position_goal.position.z = 0.0;
    position_goal.relative = true;
    // check if second motion necessary
    if (fabs(object_position_.x) > guarateed_reach) {
      position_goal_2.position.x = object_position_.x - guarateed_reach;
      position_goal_2.position.y = 0.0;
      position_goal_2.position.z = 0.0;
      position_goal_2.relative = true;
    } else {
      skip_2 = true;
    }

    skip_3 = true;
  } else {

    position_goal.position.y = 0.0;
    position_goal.position.x = 0.0;
    if (fabs(object_position_.x) < running_over_x) {
      position_goal.position.x = object_position_.x - running_over_x;
    } else {
      skip_1 = true;
    }
    position_goal.position.z = 0.0;
    position_goal.relative = true;

    // object is on the right of AOI
    if (y_diff  > 0.0) {
      object_direction_ = 2;
      position_goal_2.position.y = object_position_.y - guarateed_reach + base_offset;
      ROS_WARN("Object on right of AOI. Object at y=%f , we are moving to %f", object_position_.y, position_goal_2.position.y);
    }
    // object is on the left of AOI
    else {
      object_direction_ = 1;
      position_goal_2.position.y = object_position_.y + guarateed_reach - base_offset;
      ROS_WARN("Object on left of AOI. Object at y=%f , we are moving to %f", object_position_.y, position_goal_2.position.y);
    }
    position_goal_2.position.x = 0.0;
    position_goal_2.position.z = 0.0;
    position_goal_2.relative = true;

    // second motion
    position_goal_3.position.x = object_position_.x - base_offset;
    ROS_WARN("Object at x=%f , we are moving to %f", object_position_.x, position_goal_3.position.x);
    position_goal_3.position.y = 0.0;
    position_goal_3.position.z = 0.0;
    position_goal_3.relative = true;
  }
}


geometry_msgs::Point computeFakeMovement(double min_grasp, geometry_msgs::Point object) {
  // orientation
  geometry_msgs::Point result;
  double base_offset = 0.143;
  double dist = sqrt(pow(base_offset - object.x, 2) + pow(object.y, 2));

  double radius = dist - min_grasp; // always move a bit more than needed
  double tan_angle = atan2(object.y, object.x - base_offset);
  ROS_INFO("Computed angle: %f", tan_angle);

  result.x = radius * cos(tan_angle);
  result.y = radius * sin(tan_angle);

  ROS_INFO("Suggested movement: (%f,%f,0.0)", result.x, result.y);

  return result;
}
