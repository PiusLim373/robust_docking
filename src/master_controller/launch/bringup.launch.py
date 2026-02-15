from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # --- Include navigation2 ---
    master_controller_dir = get_package_share_directory("master_controller")
    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(master_controller_dir, "launch", "navigation2.launch.py"))
    )

    # --- Feature detector node ---
    feature_detector_node = Node(
        package="feature_detector",
        executable="feature_detector_node",
        name="feature_detector_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # --- Docking Manager node ---
    docking_manager_node = Node(
        package="docking_manager",
        executable="docking_manager_node",
        name="docking_manager_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # --- Master controller node ---
    master_controller_node = Node(
        package="master_controller",
        executable="master_controller_node.py",
        name="master_controller_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # --- Disturbance Manager node ---
    disturbance_manager_node = Node(
        package="disturbance_manager",
        executable="disturbance_manager_node",
        name="disturbance_manager_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            navigation2_launch,
            feature_detector_node,
            docking_manager_node,
            master_controller_node,
            disturbance_manager_node,
        ]
    )
