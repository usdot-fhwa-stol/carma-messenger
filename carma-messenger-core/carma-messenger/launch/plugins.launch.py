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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import FindPackageShare
from carma_ros2_utils.launch.get_log_level import GetLogLevel

import os

def generate_launch_description():
    """
    Launch file for launching the use case specific nodes being used in carma-messenger
    """

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    truck_inspection = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([FindPackageShare(), '/launch/truck_inspection_plugin_launch.py']),
                launch_arguments = { 
                    'log_level' : GetLogLevel('truck_inspection_plugin', env_log_levels),
                    }.items()
            )


    return LaunchDescription([
        env_log_levels,
        truck_inspection
    ])
