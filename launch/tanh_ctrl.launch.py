from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _config_path(filename: str) -> str:
    return str(Path(__file__).resolve().parents[1] / "config" / filename)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tanh_ctrl",
                executable="tanh_ctrl_node",
                name="tanh_ctrl",
                output="screen",
                parameters=[_config_path("tanh_ctrl.yaml")],
            )
        ]
    )
