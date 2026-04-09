from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def _package_source_root() -> Path:
    launch_file = Path(__file__).resolve()
    package_root = launch_file.parents[1]
    if (package_root / "CMakeLists.txt").exists():
        return package_root

    workspace_root = next((parent.parent for parent in launch_file.parents if parent.name == "install"), None)
    if workspace_root is None:
        return package_root

    src_root = workspace_root / "src"
    if not src_root.is_dir():
        return package_root

    for config_path in src_root.rglob("config/tanh_ctrl.yaml"):
        if (config_path.parent.parent / "launch").is_dir():
            return config_path.parent.parent

    return package_root


def _config_path(filename: str) -> str:
    return str(_package_source_root() / "config" / filename)


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
