#!/bin/bash

# gnome-terminal -t "cyberdog_gazebo" -x bash -c "source /opt/ros/galactic/setup.bash;source install/setup.bash;ros2 launch cyberdog_gazebo gazebo.launch.py rname:=l91_p1_1;exec bash;"
source /opt/ros/galactic/setup.bash
source install/setup.bash

ros2 launch cyberdog_visual cyberdog_visual.launch.py