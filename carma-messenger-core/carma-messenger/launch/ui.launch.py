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

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import SetRemap
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os


def generate_launch_description():
    """
    Launch file for launching the nodes in the CARMA Messenger user interface stack
    """

    params = os.path.join(
        get_package_share_directory('carma-messenger'),
        'ui',
        'config',
        'CommandAPIParams.yaml'
        )

    ui_group = GroupAction(
        actions=[
            SetRemap("system_alert", "/system_alert"),
            SetRemap("bsm", "incoming_bsm"),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource(os.path.join(get_package_share_directory("rosbridge_server"), 'launch', 'rosbridge_websocket_launch.xml')),
                launch_arguments = {
                    'params_glob' : [params],
                }.items()
            ),
        ]
    )

    return LaunchDescription([
        ui_group
    ])