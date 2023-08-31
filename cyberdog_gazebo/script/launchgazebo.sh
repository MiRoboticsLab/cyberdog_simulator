#!/bin/bash

source /opt/ros/galactic/setup.bash
source install/setup.bash
chmod +x src/cyberdog_simulator/cyberdog_gazebo/script/gazebolauncher.py

# ros2 launch cyberdog_gazebo gazebo.launch.py
python3 src/cyberdog_simulator/cyberdog_gazebo/script/gazebolauncher.py ros2 launch cyberdog_gazebo gazebo.launch.py