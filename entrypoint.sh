#!/bin/bash
#source ros2 and the workspace before executing any command.
#this is necessary because docker doesn't load .bashrc
set -e
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
exec "$@"
