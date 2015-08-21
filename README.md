# youbot_picknplace
Object sensing and manipulation for youbot platform


# Installation

- Clone this project
```
git clone https://github.com/ipab-rad/youbot_picknplace.git
```
- YouBot and MoveIt related configuration files (except youbot_driver)
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
- If needed, install any of the ORK deps
```
sudo apt-get install libopenni-dev ros-hydro-catkin ros-hydro-ecto* ros-hydro-opencv-candidate ros-hydro-moveit-msgs
```
- For simulations in Gazebo
```
git clone https://github.com/mas-group/youbot_simulation.git
```

- Build the project using custom made build script
```
bash catkin_make_clean.sh
```

# Usage


- ORK Tabletop
    - Add new object to DB to be detected:
   ```
    rosrun object_recognition_core object_add.py -n cube -d "A simple cube" --commit
    ```
    - Add object mesh from file. Using ID returned in previous command. other mesh files can be supplied by changing the path for the object file:
    ```
    rosrun object_recognition_core mesh_add.py d399103d741b4138f935e68ba6000829 `rospack find sensing`/data/cube.stl --commit
    ```
    - Delete object using ID:
    ```
    rosrun object_recognition_core object_delete.py d399103d741b4138f935e68ba6000829 --commit
    ```
    - Plane detection with Rviz visualization:
    ```
    roslaunch openni2_launch openni2.launch
    rosrun rviz rviz
    rosrun object_recognition_core detection -c `rospack find sensing`/conf/detection.table.ros.ork
    ```
    - Object detection with Rviz visualization:
    ```
    roslaunch openni2_launch openni2.launch
    rosrun rviz rviz
    rosrun object_recognition_core detection -c  `rospack find sensing`/conf/detection.object.ros.ork
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

- Manipulating Agent

    on youBot (in different terminals):
    ```
    roslaunch youbot_picknplace prepare.launch
    roslaunch youbot_picknplace detection.launch
    ```

    on external computer (in different terminals):
    ```
    roscore
    roslaunch youbot_picknplace start.launch
    ```

    system is ready now

    to run manipulating agent (it will wait for interest area to be published):
    ```
    rosrun agent agent 1
    ```

- Exploring Agent

    - on youBot:
    ```
    roslaunch youbot_navigation_common prepare_youbot.launch
    ```

    - on external computer (in different terminals):
    ```
    roslaunch youbot_navigation_common bringup_move_base.launch
    ```

    - system is ready now

    - to run exploring agent:
    ```
    rosrun agent agent_nav 1
    ```
