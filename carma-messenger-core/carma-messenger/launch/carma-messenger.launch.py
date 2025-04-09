# Copyright (C) 2024-2025 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python import get_package_share_directory
from launch.actions import Shutdown
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import EnvironmentVariable
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition

import os

def generate_launch_description():
    """
    Launch CARMA Messenger System.
    """

    configuration_delay = LaunchConfiguration('configuration_delay')
    declare_configuration_delay_arg = DeclareLaunchArgument(
        name ='configuration_delay', default_value='2.0')

    use_rosbag = LaunchConfiguration('use_rosbag')
    declare_use_rosbag = DeclareLaunchArgument(
        name = 'use_rosbag',
        default_value = 'true',
        description = "Record a ROS2 bag"
    )

    # Declare the route file folder launch argument
    route_file_folder = LaunchConfiguration('route_file_folder')
    declare_route_file_folder = DeclareLaunchArgument(
        name = 'route_file_folder',
        default_value='/opt/carma/routes/',
        description = 'Path of folder on host PC containing route CSV file(s) that can be accessed by plugins'
    )

    transform_group = GroupAction(
        actions=[
            PushRosNamespace(EnvironmentVariable('CARMA_TF_NS', default_value='/')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('carma-messenger'), '/launch', '/transforms.launch.py']),
            ),
        ]
    )

    v2x_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('v2x-ros-conversion'), '/launch','/v2x-ros-conversion.launch.py']),
                launch_arguments = {
                    'configuration_delay' : [configuration_delay]
                }.items()
            ),
        ]
    )

    plugins_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('carma-messenger'), '/launch', '/plugins.launch.py']),
                launch_arguments = {
                    'configuration_delay' : [configuration_delay],
                    'route_file_folder' : route_file_folder
                }.items()
            ),
        ]
    )

    ui_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('carma-messenger'), '/launch', '/ui.launch.py']),
            ),
        ]
    )

    # Launch ROS2 rosbag logging
    ros2_rosbag_group = GroupAction(
        condition = IfCondition(use_rosbag),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('carma-messenger'), '/launch', '/ros2_rosbag.launch.py']),
            )
        ]
    )

    traffic_incident_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([get_package_share_directory('traffic_incident'), '/launch', '/traffic_incident.launch.py']),
            ),
        ]
    )


    system_alert_publisher = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/system_alert', 'carma_msgs/msg/SystemAlert', '"{ type: 5, description: Simulated Drivers Ready }"']
    )

    return LaunchDescription([
        declare_configuration_delay_arg,
        declare_use_rosbag,
        declare_route_file_folder,
        transform_group,
        v2x_group,
        plugins_group,
        ui_group,
        ros2_rosbag_group,
        traffic_incident_group,
        system_alert_publisher
    ])
