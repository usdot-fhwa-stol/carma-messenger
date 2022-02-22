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
from launch_ros.substitutions import FindPackageShare
from carma_ros2_utils.launch.get_log_level import GetLogLevel
import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events 
import launch_ros.events.lifecycle
import lifecycle_msgs.msg
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():
    """
    Launch file for launching the use case specific nodes being used in carma-messenger
    """

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    configuration_delay = LaunchConfiguration('configuration_delay')
    declare_configuration_delay_arg = DeclareLaunchArgument(
        name ='configuration_delay', default_value='4.0')

    truck_inspection = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([FindPackageShare('truck_inspection_plugin'), '/launch/truck_inspection_plugin_launch.py']),
                launch_arguments = { 
                    'log_level' : GetLogLevel('truck_inspection_plugin', env_log_levels),
                    }.items()
            )
    
    ros2_cmd = launch.substitutions.FindExecutable(name='ros2')

    process_configure_truck_inspection_plugin = launch.actions.ExecuteProcess(
        cmd=[ros2_cmd, "lifecycle", "set", "/truck_inspection_plugin_node", "configure"],
    )

    configuration_trigger = launch.actions.TimerAction(
        period=configuration_delay,
        actions=[
            process_configure_truck_inspection_plugin,
        ]
    )

    configured_event_handler_truck_inspection_plugin = launch.actions.RegisterEventHandler(launch.event_handlers.OnExecutionComplete(
            target_action=process_configure_truck_inspection_plugin,
            on_completion=[ 
                launch.actions.ExecuteProcess(
                    cmd=[ros2_cmd, "lifecycle", "set", "/truck_inspection_plugin_node", "activate"],
                )
            ]
        )
    )


    return LaunchDescription([
        declare_configuration_delay_arg,
        truck_inspection,
        configuration_trigger,
        configured_event_handler_truck_inspection_plugin
    ])
