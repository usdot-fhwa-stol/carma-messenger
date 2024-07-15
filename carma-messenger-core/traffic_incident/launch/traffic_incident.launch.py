# Copyright (C) 2024 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, FindExecutable
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
from launch.event_handlers import OnExecutionComplete

import os

"""
This file is used to launch the traffic incident node.
"""


def generate_launch_description():

    # Declare the log_level launch argument
    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        name="log_level", default_value="WARN"
    )

    # Declare the configuration_delay launch argument
    configuration_delay = LaunchConfiguration('configuration_delay')
    declare_configuration_delay_arg = DeclareLaunchArgument(
        name ='configuration_delay', default_value='4.0')

    traffic_incident_param_file = os.path.join(
        get_package_share_directory("traffic_incident"), "config/parameters.yaml"
    )

    # Launch node(s) in a carma container to allow logging to be configured
    container = ComposableNodeContainer(
        package="carma_ros2_utils",
        name="traffic_incident_container",
        executable="carma_component_container_mt",
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            # Launch the core node(s)
            ComposableNode(
                package="traffic_incident",
                plugin="traffic::TrafficIncidentNode",
                name="traffic_incident",
                extra_arguments=[
                    {"use_intra_process_comms": True},
                    {"--log-level": log_level},
                ],
                parameters=[
                    traffic_incident_param_file,
                ],
                remappings=[
                    ("gps_common_fix", "/position/gps_common_fix"),
                ]
            ),
        ],
    )

    ros2_cmd = FindExecutable(name='ros2')

    process_configure_traffic_incident = ExecuteProcess(
        cmd=[ros2_cmd, "lifecycle", "set", "/traffic_incident", "configure"],
    )

    configuration_trigger = TimerAction(
        period=configuration_delay,
        actions=[
            process_configure_traffic_incident,
        ]
    )

    configured_event_handler_traffic_incident = RegisterEventHandler(OnExecutionComplete(
            target_action=process_configure_traffic_incident,
            on_completion=[ 
                ExecuteProcess(
                    cmd=[ros2_cmd, "lifecycle", "set", "/traffic_incident", "activate"],
                )
            ]
        )
    )

    return LaunchDescription(
        [
            declare_log_level_arg,
            declare_configuration_delay_arg,
            container,
            configuration_trigger,
            configured_event_handler_traffic_incident
        ]
    )