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
                namespace='action_bridge_execute_transferjob',  # corresponds to setting argument "--ros-args --remap __name:=action_bridge_execute_transferjob"
                respawn=True,
                respawn_delay=1,
                arguments=["ros2", "amr_road_network_msgs", "action/ExecuteTransferJob", "/execute_transferjob"]
            ),
            Node(
                package='ros1_bridge',
                executable='action_bridge',
                output='screen',
                name='action_bridge_execute_transferjob_flexbe',
                namespace='action_bridge_execute_transferjob_flexbe',  # corresponds to setting argument "--ros-args --remap __name:=action_bridge_execute_transferjob_flexbe"
                respawn=True,
                respawn_delay=1,
                arguments=["ros1", "amr_road_network_msgs", "ExecuteTransferJob", "/execute_transferjob_flexbe"]
            ),
        ]
    )
