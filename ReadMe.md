# CyberDog simulator

该仿真平台使用gazebo plugin的形式，消除ros通信不同步对控制的影响。同时，提供了基于Rviz2的可视化工具，将机器人状态的lcm数据转发到ROS。

推荐安装环境： Ubuntu 20.04 + ROS2 Galactic

## 下载
```
$ git clone https://github.com/MiRoboticsLab/cyberdog_sim.git
$ cd cyberdog_sim
$ vcs import < cyberdog_sim.repos
```
## 编译
需要将src/cyberdog locomotion/CMakeLists.txt中的BUILD_ROS置为ON
然后需要在cyberdog_sim文件夹下进行编译
```
$ source /opt/ros/galactic/setup.bash 
$ colcon build --merge-install --symlink-install --packages-up-to cyberdog_locomotion cyberdog_simulator
```

## 使用
需要在cyberdog_sim文件夹下运行
```
$ python3 src/cyberdog_simulator/cyberdog_gazebo/script/launchsim.py
```

### 也可以通过以下命令分别运行各程序：

首先启动gazebo程序，于cyberdog_sim文件夹下进行如下操作：
```
$ source /opt/ros/galactic/setup.bash
$ source install/setup.bash
$ ros2 launch cyberdog_gazebo gazebo.launch.py
```
也可通过如下命令打开激光雷达
```
$ source /opt/ros/galactic/setup.bash
$ source install/setup.bash
$ ros2 launch cyberdog_gazebo gazebo.launch.py use_lidar:=true
```

然后启动cyberdog_locomotion devel分支的控制程序。在cyberdog_sim文件夹下运行：
```
$ source /opt/ros/galactic/setup.bash
$ source install/setup.bash
$ ros2 launch cyberdog_gazebo cyberdog_control_launch.py
```

最后打开可视化界面，在cyberdog_sim文件夹下运行：
```
$ source /opt/ros/galactic/setup.bash
$ source install/setup.bash
$ ros2 launch cyberdog_visual cyberdog_visual.launch.py
```
