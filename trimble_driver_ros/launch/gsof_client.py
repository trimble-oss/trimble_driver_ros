#  Copyright (c) 2024. Trimble Inc.
#  All rights reserved.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('trimble_driver_ros'),
        'config',
        'gsof_client_params.yaml'
    )

    # XXX The node name has to match the one in the yaml
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='gsof_client',
            description='Name of the node'
        ),
        DeclareLaunchArgument(
            'config',
            default_value=config,
            description='YAML file for setting ROS params'
        ),
        Node(
            package='trimble_driver_ros',
            executable='gsof_client_node',
            name=LaunchConfiguration('node_name'),
            parameters=[config, LaunchConfiguration('config')],
            # any params set in the user-provided config will overwrite default
            output='screen'
        )
    ])

    return ld
