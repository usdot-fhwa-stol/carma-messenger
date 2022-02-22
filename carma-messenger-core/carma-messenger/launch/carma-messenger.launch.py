# Copyright (C) 2022 LEIDOS.
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
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    """
    Launch CARMA Messenger System.
    """

    configuration_delay = LaunchConfiguration('configuration_delay')
    declare_configuration_delay_arg = DeclareLaunchArgument(
        name ='configuration_delay', default_value='4.0')

    v2x_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/message.launch.py']),
                launch_arguments = { 
                    'configuration_delay' : [configuration_delay]
                }.items()
            ),
        ]
    )

    plugins_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/plugins.launch.py']),
                launch_arguments = { 
                    'configuration_delay' : [configuration_delay]
                }.items()
            ),
        ]
    )

    return LaunchDescription([
        declare_configuration_delay_arg,
        v2x_group,
        plugins_group
    ])
