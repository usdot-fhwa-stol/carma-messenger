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
                    ("inbound_binary_msg", "comms/inbound_binary_msg" ),
                    ("outbound_binary_msg", "comms/outbound_binary_msg" ),
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

    # Use emit event actions to trigger the lifecycle state transitions
    emit_event_cpp_message_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(cpp_message_component),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    emit_event_j2735_convertor_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(j2735_convertor_component),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    register_event_handler_for_cpp_message_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=cpp_message_component, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'cpp_message_component' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(cpp_message_component),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    register_event_handler_for_j2735_convertor_inactive = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=j2735_convertor_component, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'j2735_convertor_component' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(j2735_convertor_component),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([ 
        carma_v2x_container,
        register_event_handler_for_cpp_message_inactive,
        register_event_handler_for_j2735_convertor_inactive,
        emit_event_cpp_message_configure,
        emit_event_j2735_convertor_configure
    ]) 

