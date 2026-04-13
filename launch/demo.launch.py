from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _config_root() -> Path:
    package_root = Path(__file__).resolve().parents[1]
    if (package_root / "CMakeLists.txt").exists():
        return package_root / "config"

    workspace_root = next((parent.parent for parent in Path(__file__).resolve().parents if parent.name == "install"), None)
    if workspace_root is None:
        return package_root / "config"

    source_config = workspace_root / "src" / "TanhCtrl" / "config"
    if source_config.is_dir():
        return source_config

    return package_root / "config"


def _config_path(filename: str) -> str:
    return str(_config_root() / filename)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tanh_ctrl",
                executable="tanh_ctrl_node",
                name="tanh_ctrl",
                output="screen",
                parameters=[_config_path("tanh_ctrl.yaml")],
            ),
            Node(
                package="tanh_ctrl",
                executable="flatness_reference_publisher.py",
                name="flatness_reference_publisher",
                output="screen",
                parameters=[_config_path("flatness_reference.yaml")],
            ),
        ]
    )
