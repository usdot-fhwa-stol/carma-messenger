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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from carma_ros2_utils.launch.get_log_level import GetLogLevel

import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle
import lifecycle_msgs.msg


def generate_launch_description():
    """
    Launch desired CARMA Messenger drivers
    """

    env_log_levels = EnvironmentVariable(
        "CARMA_ROS_LOGGING_CONFIG", default_value='{ "default_level" : "WARN" }'
    )

    configuration_delay = LaunchConfiguration("configuration_delay")
    declare_configuration_delay_arg = DeclareLaunchArgument(
        name="configuration_delay", default_value="4.0"
    )

    drivers = LaunchConfiguration("drivers")
    declare_drivers_arg = DeclareLaunchArgument(
        name="drivers",
        default_value="v2x_ros_driver",
        description="Desired drivers to launch specified by package name.",
    )

    v2x_ros_driver_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'v2x_ros_driver' in '", drivers, "'.split()"])
        ),
        actions=[
            PushRosNamespace(
                EnvironmentVariable("CARMA_INTR_NS", default_value="hardware_interface")
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("v2x_ros_driver"), "/launch/v2x_ros_driver.launch.py"]
                ),
                launch_arguments={
                    "log_level": GetLogLevel("v2x_ros_driver", env_log_levels),
                }.items(),
            ),
        ],
    )

    pinpoint_group = GroupAction(
        condition=IfCondition(
            PythonExpression(["'pinpoint_driver' in '", drivers, "'.split()"])
        ),
        actions=[
            PushRosNamespace(
                EnvironmentVariable("CARMA_INTR_NS", default_value="hardware_interface")
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("pinpoint"), "/launch/pinpoint.launch.py"]
                ),
                launch_arguments={
                    "log_level": GetLogLevel("pinpoint", env_log_levels),
                }.items(),
            ),
        ],
    )

    ros2_cmd = launch.substitutions.FindExecutable(name="ros2")

    process_configure_v2x_ros_driver_node = launch.actions.ExecuteProcess(
        condition=IfCondition(
            PythonExpression(["'v2x_ros_driver' in '", drivers, "'.split()"])
        ),
        cmd=[
            ros2_cmd,
            "lifecycle",
            "set",
            [
                EnvironmentVariable(
                    "CARMA_INTR_NS", default_value="hardware_interface"
                ),
                "/v2x_ros_driver_node",
            ],
            "configure",
        ],
    )

    configured_event_handler_v2x_ros_driver_node = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnExecutionComplete(
            target_action=process_configure_v2x_ros_driver_node,
            on_completion=[
                launch.actions.ExecuteProcess(
                    cmd=[
                        ros2_cmd,
                        "lifecycle",
                        "set",
                        [
                            EnvironmentVariable(
                                "CARMA_INTR_NS", default_value="hardware_interface"
                            ),
                            "/v2x_ros_driver_node",
                        ],
                        "activate",
                    ],
                )
            ],
        )
    )

    process_configure_pinpoint_node = launch.actions.ExecuteProcess(
        condition=IfCondition(
            PythonExpression(["'pinpoint_driver' in '", drivers, "'.split()"])
        ),
        cmd=[
            ros2_cmd,
            "lifecycle",
            "set",
            [
                EnvironmentVariable(
                    "CARMA_INTR_NS", default_value="hardware_interface"
                ),
                "/pinpoint",
            ],
            "configure",
        ],
    )

    configured_event_handler_pinpoint_node = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnExecutionComplete(
            target_action=process_configure_pinpoint_node,
            on_completion=[
                launch.actions.ExecuteProcess(
                    cmd=[
                        ros2_cmd,
                        "lifecycle",
                        "set",
                        [
                            EnvironmentVariable(
                                "CARMA_INTR_NS", default_value="hardware_interface"
                            ),
                            "/pinpoint",
                        ],
                        "activate",
                    ],
                )
            ],
        )
    )

    configuration_trigger = launch.actions.TimerAction(
        period=configuration_delay,
        actions=[process_configure_v2x_ros_driver_node, process_configure_pinpoint_node],
    )

    return LaunchDescription(
        [
            declare_configuration_delay_arg,
            declare_drivers_arg,
            v2x_ros_driver_group,
            pinpoint_group,
            configuration_trigger,
            configured_event_handler_v2x_ros_driver_node,
            configured_event_handler_pinpoint_node,
        ]
    )
