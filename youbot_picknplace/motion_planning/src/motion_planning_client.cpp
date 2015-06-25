#include "ros/ros.h"
#include "motion_planning_msgs/PlanMotion.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_motion_client");
  if (argc != 4)
  {
    ROS_INFO("usage: plan_motion_client X Y Z");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<motion_planning_msgs::PlanMotion>("plan_motion");
  motion_planning_msgs::PlanMotion srv;
  srv.request.x = atof(argv[1]);
  srv.request.y = atof(argv[2]);
  srv.request.z = atof(argv[3]);
  if (client.call(srv))
  {
    ROS_INFO("Success: %f", (double)srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service plan_motion");
    return 1;
  }

  return 0;
}