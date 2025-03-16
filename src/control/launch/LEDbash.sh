#!/bin/bash

# Properly source ROS environment for the current session
source /opt/ros/noetic/setup.bash
source ~/Zpice/devel/setup.bash

# Run the Python script with sudo and environment variables
echo "    " | sudo -S -E env "PYTHONPATH=$PYTHONPATH" python3 $(rospack find control)/src/script/LEDDriverNode.py
echo "    " | sudo -S -E env "PYTHONPATH=$PYTHONPATH" python3 $(rospack find control)/src/script/API/servers/MissionColorServer.py


