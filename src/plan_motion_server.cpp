#include "ros/ros.h"
#include "youbot_picknplace/PlanMotion.h"
#include <pluginlib/class_loader.h>

 //tf
 #include <tf/transform_datatypes.h>

// moveit interfaces
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>

bool planMotion(youbot_picknplace::PlanMotion::Request  &req,
         youbot_picknplace::PlanMotion::Response &res)
{

  ros::NodeHandle node_handle;

  // load robot descrpition from ros param server
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  // load planning scene
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // planner manager
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

  // load plugin for motion planning
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch(pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0 ; i < classes.size() ; ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                     << "Available plugins: " << ss.str());
  }

  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the PR2
  // specifying the desired pose of the end-effector as input.
  planning_interface::MotionPlanRequest request;
  planning_interface::MotionPlanResponse response;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_footprint";
  geometry_msgs::Quaternion quat ;
  quat = tf::createQuaternionMsgFromRollPitchYaw(-3.129,0.0549,1.686);
  pose.pose.orientation.x = quat.x;
  pose.pose.orientation.y = quat.y;
  pose.pose.orientation.z = quat.z;
  pose.pose.orientation.w = quat.w;
  ROS_INFO("Quaternion info- x: %f  y: %f  z: %f  w: %f", quat.x, quat.y, quat.z, quat.w);
  pose.pose.position.x = req.x;
  pose.pose.position.y = req.y;
  pose.pose.position.z = req.z;

  // A tolerance of 0.1 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.1);
  std::vector<double> tolerance_angle(3, 0.01);

  request.group_name = "arm_1";
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("arm_link_5", pose, tolerance_pose, tolerance_angle);
  request.goal_constraints.push_back(pose_goal);

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this 
  // planning context
  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, request, response.error_code_);
  context->solve(response);
  if(response.error_code_.val != response.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    res.success = 0;
    return true;
  }else{
    ROS_INFO("Planned computed successfully");
    res.success = 1;
  }

  moveit_msgs::MotionPlanResponse configuration_response;
  response.getMessage(configuration_response);
  std::vector<double> joint_values = configuration_response.trajectory.joint_trajectory.points.back().positions;
  ROS_INFO("Final joint configuration to achieve requested pose (radians):");
  for(int i=0; i < joint_values.size(); ++i)
  {
    ROS_INFO("Joint %d: %f", i, joint_values[i]);
  }

  ROS_INFO("request: x=%f, y=%f, z=%f", (double)req.x, (double)req.y, (double)req.z);
  ROS_INFO("sending back response: [%f]", (double) res.success);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_motion_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("plan_motion", planMotion);
  ROS_INFO("Ready to plan motion.");
  ros::spin();

  return 0;
}