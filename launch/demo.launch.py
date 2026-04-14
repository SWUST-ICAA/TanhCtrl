from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_root = Path(__file__).resolve().parents[1]
    controller_config = package_root / "config" / "tanh_ctrl.yaml"
    reference_config = package_root / "config" / "flatness_reference.yaml"

    return LaunchDescription(
        [
            Node(
                package="tanh_ctrl",
                executable="tanh_ctrl_node",
                name="tanh_ctrl",
                output="screen",
                parameters=[str(controller_config)],
            ),
            Node(
                package="tanh_ctrl",
                executable="flatness_reference_publisher.py",
                name="flatness_reference_publisher",
                output="screen",
                parameters=[str(reference_config)],
            ),
        ]
    )
