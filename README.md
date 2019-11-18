## ENPM808X Obstacle Avoidance Robot
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
Implementation of a simple walker algorithm like a Roomba robot vacuum cleaner.The robot moves forward until it reaches an obstacle (but not colliding), then rotates in place until the way ahead is clear, then moves forward again and repeat.

## Dependencies
- Ubuntu 16.04
- ROS kinetic
- catkin
- Turtlebot packages

To install ROS, follow the instructions on this [link](http://wiki.ros.org/kinetic/Installation)

To install turtlebot packages run following command.
```
sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
```
## Build Instructions

```
source /opt/ros/kinetic/setup.bash 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Ip-umd/turtlebot3_walker.git
cd ..
catkin_make
```

## Running the code
Running the following instructions will open a gazebo environment with turtlebot3 and the turtlebot3 will navigate in the world avoiding obstacles. Placing obstacles in front of the moving turtlebot3 will better show the functionality of obstacle avoidance algorithm. Following command will only execute the simulation:

```
cd <path to catkin_ws>
source devel/setup.bash
roslaunch turtlebot3_walker turtlebot3_walker.launch
```

To run the simulation alongwith recording a rosbag, execute the following command:
```
roslaunch turtlebot3_walker turtlebot3_walker.launch rosbag_record:=true
```

To disable bag file recording, pass false to the argument rosbag_record i.e.  "rosbag_record:=false"

### Inspecting the bag file
Run the following command to inspect the bag file:
```
cd <path to repository>/results
rosbag info walker.bag
```
It should give similar output as following:
```
path:        walker.bag
version:     2.0
duration:    29.9s
start:       Dec 31 1969 19:00:00.29 (0.29)
end:         Dec 31 1969 19:00:30.21 (30.21)
size:        11.7 MB
messages:    23041
compression: none [16/16 chunks]
types:       bond/Status                           [eacc84bf5d65b6777d4c50f463dfb9c8]
             dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             std_msgs/String                       [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                                            2995 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel_mux/active                                  1 msg     : std_msgs/String                      
             /cmd_vel_mux/input/navi                            302 msgs    : geometry_msgs/Twist                  
             /cmd_vel_mux/parameter_descriptions                  1 msg     : dynamic_reconfigure/ConfigDescription
             /cmd_vel_mux/parameter_updates                       1 msg     : dynamic_reconfigure/Config           
             /depthimage_to_laserscan/parameter_descriptions      1 msg     : dynamic_reconfigure/ConfigDescription
             /depthimage_to_laserscan/parameter_updates           1 msg     : dynamic_reconfigure/Config           
             /gazebo/link_states                               2988 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states                              2988 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions                       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates                            1 msg     : dynamic_reconfigure/Config           
             /gazebo_gui/parameter_descriptions                   1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo_gui/parameter_updates                        1 msg     : dynamic_reconfigure/Config           
             /joint_states                                     2871 msgs    : sensor_msgs/JointState               
             /laserscan_nodelet_manager/bond                     58 msgs    : bond/Status                           (2 connections)
             /mobile_base/commands/velocity                     286 msgs    : geometry_msgs/Twist                  
             /mobile_base/sensors/imu_data                     2873 msgs    : sensor_msgs/Imu                      
             /mobile_base_nodelet_manager/bond                  112 msgs    : bond/Status                           (3 connections)
             /odom                                             2872 msgs    : nav_msgs/Odometry                    
             /rosout                                            353 msgs    : rosgraph_msgs/Log                     (10 connections)
             /rosout_agg                                        336 msgs    : rosgraph_msgs/Log                    
             /scan                                              283 msgs    : sensor_msgs/LaserScan                
             /tf                                               3714 msgs    : tf2_msgs/TFMessage                    (2 connections)
             /tf_static                                           1 msg     : tf2_msgs/TFMessage

```
### Playing back the bag file 
Play back the bag file using following command:
```
cd <path to repository>/results
rosbag play walker.bag
```
Verify the output using following command:
```
source /opt/ros/kinetic/setup.bash
rostopic echo /cmd_vel_mux/input/navi
```
