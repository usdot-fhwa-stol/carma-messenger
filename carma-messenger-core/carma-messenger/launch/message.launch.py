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

from launch.actions import Shutdown
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable
from carma_ros2_utils.launch.get_log_level import GetLogLevel
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events 
import launch_ros.events.lifecycle
import lifecycle_msgs.msg


def generate_launch_description():
    """
    Launch V2X subsystem nodes.
    """

    env_log_levels = EnvironmentVariable('CARMA_ROS_LOGGING_CONFIG', default_value='{ "default_level" : "WARN" }')

    configuration_delay = LaunchConfiguration('configuration_delay')
    declare_configuration_delay_arg = DeclareLaunchArgument(
        name ='configuration_delay', default_value='4.0')
    
     # Nodes
    cpp_message_component = ComposableNode(
                package='cpp_message',
                plugin='cpp_message::Node',
                name='cpp_message_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : GetLogLevel('cpp_message', env_log_levels) }
                ],
                remappings=[
                    ("inbound_binary_msg", "hardware_interface/comms/inbound_binary_msg" ),
                    ("outbound_binary_msg", "hardware_interface/comms/outbound_binary_msg" ),
                ],
            )
    
    j2735_convertor_component = ComposableNode(
                package='j2735_convertor',
                plugin='j2735_convertor::Node',
                name='j2735_convertor_node',
                extra_arguments=[
                    {'use_intra_process_comms': True},
                    {'--log-level' : GetLogLevel('j2735_convertor', env_log_levels) }
                ],
                remappings=[
                    ("outgoing_bsm", "bsm_outbound" ),
                ],
            )

    carma_v2x_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='carma_v2x_container',
        executable='carma_component_container_mt',
        namespace=GetCurrentNamespace(),
        composable_node_descriptions=[
            cpp_message_component,
            j2735_convertor_component,
        ],
        on_exit= Shutdown()
    )

    ros2_cmd = launch.substitutions.FindExecutable(name='ros2')

    process_configure_cpp_message = launch.actions.ExecuteProcess(
        cmd=[ros2_cmd, "lifecycle", "set", "/cpp_message_node", "configure"],
    )

    process_configure_j2735_convertor = launch.actions.ExecuteProcess(
        cmd=[ros2_cmd, "lifecycle", "set", "/j2735_convertor_node", "configure"],
    )

    configuration_trigger = launch.actions.TimerAction(
        period=configuration_delay,
        actions=[
            process_configure_cpp_message,
            process_configure_j2735_convertor
        ]
    )

    configured_event_handler_cpp_message = launch.actions.RegisterEventHandler(launch.event_handlers.OnExecutionComplete(
            target_action=process_configure_cpp_message,
            on_completion=[ 
                launch.actions.ExecuteProcess(
                    cmd=[ros2_cmd, "lifecycle", "set", "/cpp_message_node", "activate"],
                )
            ]
        )
    )

    configured_event_handler_j2735_convertor = launch.actions.RegisterEventHandler(launch.event_handlers.OnExecutionComplete(
        target_action=process_configure_cpp_message, on_completion=[ 
            launch.actions.ExecuteProcess(
                cmd=[ros2_cmd, "lifecycle", "set", "/j2735_convertor_node", "activate"],
            )
        ])
    )

    return LaunchDescription([ 
        declare_configuration_delay_arg,
        carma_v2x_container,
        configuration_trigger,
        configured_event_handler_cpp_message,
        configured_event_handler_j2735_convertor
    ]) 

