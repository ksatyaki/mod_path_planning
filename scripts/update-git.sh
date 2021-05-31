#!/usr/bin/env zsh

source /opt/ros/kinetic/setup.zsh
source ~/workspace/cpp_ws/devel/setup.zsh

roscd mod_path_planning
git pull origin master
roscd cliffmap_ros
git pull origin master
roscd ompl_planners_ros
git pull origin master
cd ~/workspace/cpp_ws
catkin_make

echo "Up-to-date"
