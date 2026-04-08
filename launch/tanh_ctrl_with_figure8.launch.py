from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _source_config(package: str, filename: str) -> str:
    workspace_src = Path(__file__).resolve().parents[2]
    return str(workspace_src / package / "config" / filename)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="figure8_path_publisher",
                executable="figure8_path_publisher_node",
                name="figure8_path_publisher",
                output="screen",
                parameters=[_source_config("figure8_path_publisher", "figure8_tanh_ctrl.yaml")],
            ),
            Node(
                package="tanh_ctrl",
                executable="tanh_ctrl_node",
                name="tanh_ctrl",
                output="screen",
                parameters=[_source_config("tanh_ctrl", "tanh_ctrl.yaml")],
            ),
        ]
    )
