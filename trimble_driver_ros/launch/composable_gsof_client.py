#  Copyright (c) 2024. Trimble Inc.
#  All rights reserved.

# Just an example composition launch file to show that gsof_client is composable.
# This file is mostly useless on its own.
import launch
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('trimble_driver_ros'),
        'config',
        'gsof_client_params.yaml'
    )

    arg_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='gsof_client',
        description='Name of the node'
    )

    arg_config = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='YAML file for setting ROS params'
    )

    # Parse default config file into parameter dictionary
    with open(default_config, 'r') as f:
        default_params = yaml.safe_load(f)['gsof_client']['ros__parameters']

    container = ComposableNodeContainer(
            name='trimble_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='trimble_driver_ros',
                    plugin='trimble_driver_ros::GsofClientRos',
                    name=LaunchConfiguration('node_name'),
                    parameters=[default_params, LaunchConfiguration('config')])
            ],
            output='screen',
    )

    return launch.LaunchDescription([arg_node_name, arg_config, container])
