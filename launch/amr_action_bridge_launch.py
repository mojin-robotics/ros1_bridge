# Copyright 2023 4am Robotics GmbH
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
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='ros1_bridge',
                executable='action_bridge',
                output='screen',
                name='action_bridge_execute_transferjob',
                namespace='action_bridge_execute_transferjob',
                respawn=True,
                respawn_delay=1,
                arguments=['ros2',
                           'amr_road_network_msgs',
                           'action/ExecuteTransferJob',
                           '/execute_transferjob']
            ),
            Node(
                package='ros1_bridge',
                executable='action_bridge',
                output='screen',
                name='action_bridge_execute_transferjob_flexbe',
                namespace='action_bridge_execute_transferjob_flexbe',
                respawn=True,
                respawn_delay=1,
                arguments=['ros1',
                           'amr_road_network_msgs',
                           'ExecuteTransferJob',
                           '/execute_transferjob_flexbe']
            ),
        ]
    )
