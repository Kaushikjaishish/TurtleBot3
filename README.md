# TurtleBot3
Autonomous random navigation node for TurtleBot3 using Navigation2 (Nav2) in Gazebo.
The robot continuously selects random goals and navigates without colliding with obstacles.

https://youtu.be/To6L7cOUJcg

# turtlebot3_nav2_autorun

Autonomous explorer node for TurtleBot3 using Navigation2.
Selects random goals in free cells (uses /map OccupancyGrid), sends NavigateToPose goals to Nav2.

## Requirements
- Ubuntu (22.04)
- ROS2 Humble
- turtlebot3 packages, turtlebot3_gazebo, nav2_bringup installed
- colcon build tool

## Files
- auto_move.py : main node
- setup.py, package.xml, setup.cfg : package files

## Build
cd ~/ros2_ws/src
git clone (https://github.com/Kaushikjaishish/TurtleBot3)

## Build and source:
cd ~/ros2_ws
colcon build --packages-select turtlebot3_nav2_autorun
source install/setup.bash

## Set TurtleBot3 model:
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc


## Run
## Start Gazebo world:
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

## Launch Nav2:
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true

## Run the AutoMove node:
cd ~/ros2_ws
ros2 run turtlebot3_nav2_autorun auto_move
