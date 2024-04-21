import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import launch_params, robot_state_publisher, node_params, tracker_node ,energy_tracker_node
    from launch_ros.actions import Node
    from launch import LaunchDescription

    detector_node = Node(
        package='armor_detector',
        executable='armor_detector_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'armor_detector:='+launch_params['detector_log_level']],
    )
    energy_detector_node = Node(
        package='energy_detector',
        executable='energy_detector_node',
        emulate_tty=True,
        output='both',
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                   'energy_detector:='+launch_params['energy_detector_log_level']],
    )

    return LaunchDescription([
        robot_state_publisher,
        detector_node,
        tracker_node,
        energy_detector_node,
        energy_tracker_node,
    ])
