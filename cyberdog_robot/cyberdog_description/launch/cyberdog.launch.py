
import os
import launch
import xacro
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def launch_setup(context, *args, **kwargs):
    # package
    pkg_dir = get_package_share_directory('cyberdog')

    # urdf
    xacro_path = os.path.join(get_package_share_directory(
        'cyberdog'), 'xacro', 'robot.xacro')
    urdf_contents = xacro.process_file(xacro_path).toprettyxml(indent='  ')

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[
            {
                'robot_description': urdf_contents
            }
        ]
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            },
            {
                'robot_description': urdf_contents
            },
            {
                'publish_frequency': 500.0
            }
        ]
    )
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [os.path.join(pkg_dir, 'rviz', 'cyberdog.rviz')]]
    )

    return [joint_state_publisher_node, robot_state_publisher_node, rviz2_node]

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        OpaqueFunction(function=launch_setup)
    ])
