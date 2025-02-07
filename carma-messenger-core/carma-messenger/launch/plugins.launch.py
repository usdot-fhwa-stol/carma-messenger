# Copyright (C) 2022-2023 LEIDOS.
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
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
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

    route_file_folder = LaunchConfiguration('route_file_folder')

    configuration_delay = LaunchConfiguration('configuration_delay')
    declare_configuration_delay_arg = DeclareLaunchArgument(
        name ='configuration_delay', default_value='4.0')

    emergency_response_vehicle_plugin_param_file = os.path.join(
        get_package_share_directory('emergency_response_vehicle_plugin'), 'config/parameters.yaml')

    carma_v2x_plugins_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_v2x_plugins_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            ComposableNode(
                package='emergency_response_vehicle_plugin',
                plugin='emergency_response_vehicle_plugin::EmergencyResponseVehiclePlugin',
                name='emergency_response_vehicle_plugin_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : GetLogLevel('emergency_response_vehicle_plugin', env_log_levels) }
                ],
                remappings=[
                    ("vehicle_pose", "hardware_interface/gps_common_fix" ),
                    ("velocity",     "hardware_interface/velocity" ),
                    ("outgoing_bsm", "bsm_outbound")
                ],
                parameters = [
                    {'route_file_folder': route_file_folder},
                    emergency_response_vehicle_plugin_param_file
                ]
            )
        ],
        on_exit = Shutdown()
    )

    ros2_cmd = launch.substitutions.FindExecutable(name='ros2')

    process_configure_emergency_response_vehicle_plugin = launch.actions.ExecuteProcess(
        cmd=[ros2_cmd, "lifecycle", "set", "/emergency_response_vehicle_plugin_node", "configure"],
    )

    configuration_trigger = launch.actions.TimerAction(
        period=configuration_delay,
        actions=[
            process_configure_emergency_response_vehicle_plugin
        ]
    )

    configured_event_handler_emergency_response_vehicle_plugin = launch.actions.RegisterEventHandler(launch.event_handlers.OnExecutionComplete(
            target_action=process_configure_emergency_response_vehicle_plugin,
            on_completion=[
                launch.actions.ExecuteProcess(
                    cmd=[ros2_cmd, "lifecycle", "set", "/emergency_response_vehicle_plugin_node", "activate"],
                )
            ]
        )
    )


    return LaunchDescription([
        declare_configuration_delay_arg,
        carma_v2x_plugins_container,
        configuration_trigger,
        configured_event_handler_emergency_response_vehicle_plugin
    ])
