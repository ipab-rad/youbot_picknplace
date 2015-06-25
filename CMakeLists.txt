cmake_minimum_required(VERSION 2.8)
project(youbot_picknplace)

find_package(catkin REQUIRED COMPONENTS object_recognition_msgs pluginlib 
	rosconsole roscpp shape_msgs tf message_generation moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib
	std_msgs
  	geometry_msgs
  	actionlib_msgs)



add_service_files(
  FILES
  PlanMotion.srv
)
add_message_files(
  FILES
)
add_action_files(
  DIRECTORY action
  FILES
  MoveToPosition.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime moveit_core moveit_ros_planning moveit_ros_planning_interface actionlib_msgs geometry_msgs)


set(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE 1)

# include some directories
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
                    include
)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})

add_executable(object_listener 
  src/object_listener.cpp 
  )

add_executable(plan_motion_server 
  src/plan_motion_server.cpp 
  )

add_executable(plan_motion_client 
  src/plan_motion_client.cpp
  )

add_executable(move_to_position 
  src/move_to_position.cpp
  )

add_executable(move_to_position_client 
  src/move_to_position_client.cpp
  )

target_link_libraries(object_listener ${catkin_LIBRARIES})
add_dependencies(object_listener youbot_picknplace_gencpp)

target_link_libraries(plan_motion_server ${catkin_LIBRARIES})
add_dependencies(plan_motion_server youbot_picknplace_gencpp)

target_link_libraries(plan_motion_client ${catkin_LIBRARIES})
add_dependencies(plan_motion_client youbot_picknplace_gencpp)

target_link_libraries(move_to_position ${catkin_LIBRARIES})
add_dependencies(move_to_position ${youbot_picknplace_EXPORTED_TARGETS})

target_link_libraries(move_to_position_client ${catkin_LIBRARIES})
add_dependencies(move_to_position_client ${youbot_picknplace_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
