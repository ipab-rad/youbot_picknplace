#include "ros/ros.h"
#include "object_recognition_msgs/RecognizedObjectArray.h"
#include "object_recognition_msgs/RecognizedObject.h"
#include <tf/transform_listener.h>


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void recognizedCallback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
{
  tf::StampedTransform transform;
  tf::TransformListener listener;

  for (size_t i_msg = 0; i_msg < msg->objects.size(); ++i_msg) {
    const object_recognition_msgs::RecognizedObject& object = msg->objects[i_msg];
    const geometry_msgs::Point point = object.pose.pose.pose.position;
    // const object_recognition_msgs::ObjectType type = object.type;
        
    ROS_INFO("Object was recognized. Key: %s", object.type.key.c_str());    
    try{
         ROS_INFO("Frame ID: %s", object.pose.header.frame_id.c_str());
         // listener.lookupTransform("/camera_link", "/camera_rgb_frame", ros::Time(0), transform);
        // ROS_INFO("All frames: %s",listener.allFramesAsString ().c_str());
        try {
            listener.waitForTransform("/base_footprint", object.pose.header.frame_id.c_str(), ros::Time(0), ros::Duration(13.0) );
            
            // HACK: it seems coordinates z-y are flipped
            tf::Vector3 vec(point.x,point.z,point.y);
            tf::Stamped<tf::Vector3> pin(vec, ros::Time(0), object.pose.header.frame_id.c_str());
            tf::Stamped<tf::Vector3> pout;
            listener.transformVector("/base_footprint",pin,pout);
            listener.lookupTransform("/base_footprint", object.pose.header.frame_id.c_str(), ros::Time(0), transform);
            // HACK CONTINUED:
            ROS_INFO("3D point in frame: %s, Point (x,y,z): (%f,%f,%f)", object.pose.header.frame_id.c_str(),point.x, point.z, point.y);    
            // ROS_INFO("Computed transform to /base_footprint, Point (x,y,z): (%f,%f,%f)", transform.getOrigin().x(), transform.getOrigin().y(),transform.getOrigin().z());
            ROS_INFO("3D point in frame of /base_footprint, Point (x,y,z): (%f,%f,%f)", pout.getX(), -pout.getY(), pout.getZ());
            ROS_INFO("");
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }        
      }
    catch (tf::TransformException ex){
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
       }

  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_listener");

  ros::NodeHandle n;
  
  tf::TransformListener listener;


  ros::Subscriber sub = n.subscribe("/recognized_object_array", 1000, recognizedCallback);

  ros::spin();

  return 0;
}