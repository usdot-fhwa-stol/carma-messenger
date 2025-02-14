# Copyright (C) 2024 LEIDOS.
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

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from datetime import datetime
import pathlib
import yaml

# This function is used to generate a command to record a ROS 2 rosbag that excludes topics
# topics as provided in the appropriate configuration file.
def record_ros2_rosbag(context: LaunchContext, rosbag2_qos_override_param_file):

    overriding_qos_profiles = context.perform_substitution(rosbag2_qos_override_param_file)

    proc = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--qos-profile-overrides-path', overriding_qos_profiles, '-o', '/opt/carma/logs/rosbag2_' + str(datetime.now().strftime('%Y-%m-%d_%H%M%S')), '-a'],
            output='screen',
            shell='true'
        )

    return [proc]


def generate_launch_description():
    rosbag2_qos_override_param_file = LaunchConfiguration('rosbag2_qos_override_param_file')
    declare_rosbag2_qos_override_param_file = DeclareLaunchArgument(
        name='rosbag2_qos_override_param_file',
        default_value = PathJoinSubstitution([
                    FindPackageShare('carma-messenger'),'config',
                    'rosbag2_qos_overrides.yaml'
                ]),
        description = "Path to file containing rosbag2 override qos settings"
    )

    return LaunchDescription([
        declare_rosbag2_qos_override_param_file,
        OpaqueFunction(function=record_ros2_rosbag, args=[rosbag2_qos_override_param_file])
    ])