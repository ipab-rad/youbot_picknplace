# youbot_picknplace
Object sensing and manipulation for youbot platform


# Installation

- clone this project
```
git clone https://github.com/ipab-rad/youbot_picknplace.git
```
- youBot and MoveIt related configuration files (except youbot_driver)
```
git clone https://github.com/ipab-rad/rad_youbot_stack.git
```
- ROS deps
```
sudo apt-get install ros-hydro-cmake-modules
```
- MoveIt and youBot arm kinematics plugin
```
sudo apt-get install ros-hydro-moveit-full ros-hydro-moveit-simple-controller-manager
git clone https://github.com/svenschneider/youbot-manipulation.git
```
(and moveit_fake_controller_manager if needed)

- ASUS Xtion and (ORK) object detection deps
```
sudo apt-get install ros-hydro-openni2-launch ros-hydro-openni2-camera
sudo apt-get install ros-hydro-object-recognition-core ros-hydro-object-recognition-msgs ros-hydro-object-recognition-ros ros-hydro-object-recognition-ros-visualization
sudo apt-get install ros-hydro-object-recognition-tabletop
```
- if needed, install any of the ORK deps
```
sudo apt-get install libopenni-dev ros-hydro-catkin ros-hydro-ecto* ros-hydro-opencv-candidate ros-hydro-moveit-msgs
```
- for simulations in Gazebo
```
git clone https://github.com/mas-group/youbot_simulation.git
```

# Usage


- ORK Tabletop
    - add new object to DB to be detected:
   ```
    rosrun object_recognition_core object_add.py -n cube -d "A simple cube" --commit
    ```
    - add object mesh from file using ID returned in previous command:
    ```
    rosrun object_recognition_core mesh_add.py d399103d741b4138f935e68ba6000829 `rospack find object_recognition_tutorials`/data/cube.stl --commit
    ```
    - plane detection with Rviz visualization:
    ```
    roslaunch openni2_launch openni2.launch
    rosrun rviz rviz
    rosrun object_recognition_core detection -c `rospack find youbot_picknplace`/sensing/conf/detection.table.ros.ork
    ```
    -object detection with Rviz visualization:
    ```
    roslaunch openni2_launch openni2.launch
    rosrun rviz rviz
    rosrun object_recognition_core detection -c  `rospack find youbot_picknplace`/sensing/conf/detection.object.ros.ork
    ```

    NOTE: more information on the usage and FAQS of the Tabletop pipeline can be found in
    http://wg-perception.github.io/ork_tutorials/tutorial02/tutorial.html


- Gazebo Simulation
    ```
    roslaunch youbot_gazebo_robot youbot.launch
    ```


- Motion
    - MoveIt! motion execution python script
   ```
    roslaunch youbot_moveit_config move_group.launch
    rosrun moveit_commander moveit_commander_cmdline.py
```

    - Joint commander without MoveIt
```
    rosrun youbot_driver_ros_interface youbot_keyboard_arm_teleop.py
```
