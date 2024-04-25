# Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#      http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import launch
import xacro
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # config
    hang_robot = LaunchConfiguration('hang_robot').perform(context)
    use_lidar = LaunchConfiguration('use_lidar').perform(context)
    rname = LaunchConfiguration('rname').perform(context)
    use_sim_time=LaunchConfiguration('use_sim_time')

    # path
    description_share = FindPackageShare(
        package=rname+'_description').find(rname+'_description')
    visual_share = FindPackageShare(
        package='cyberdog_visual').find('cyberdog_visual')

    # urdf
    xacro_path = os.path.join(description_share, 'xacro/robot.xacro')
    urdf_contents = xacro.process_file(xacro_path, mappings={
                                       'DEBUG': hang_robot, 'USE_LIDAR': use_lidar}).toprettyxml(indent='  ')

    # joint_state_publisher
    joint_state_node = Node(
        package='cyberdog_visual',
        executable='cyberdog_visual',
        name='cyberdog_visual',
        output='screen',
        parameters=[{
                    'robot_description': urdf_contents,
                    'publish_frequency': 500.0,
                    'joint_state_topic': 'joint_states',
                    'use_sim_time': use_sim_time
                    }
                    ]
    )

    # robot_state_publisher
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                {
                    'robot_description': urdf_contents,
                    'publish_frequency': 500.0,
                    'use_sim_time': use_sim_time
                }
        ]
    )

    # rviz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [os.path.join(visual_share, 'rviz', 'cyberdog_visual2.rviz')]]
    )

    return [joint_state_node, robot_state_node, rviz2_node]

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='hang_robot',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='use_lidar',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='rname',
            default_value='cyberdog'
        ),
        OpaqueFunction(function=launch_setup)
    ])
