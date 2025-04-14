#!/bin/bash

# Properly source ROS environment for the current session
source /opt/ros/noetic/setup.bash
source ~/Zpice/devel/setup.bash

# Run the Python script with sudo and environment variables
sudo -E env "PYTHONPATH=$PYTHONPATH" python3 $(rospack find control)/src/script/PressureNode.py

