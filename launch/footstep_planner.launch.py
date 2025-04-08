import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare("baseline_footstep_planner").find("baseline_footstep_planner")
    rviz_config_file = os.path.join(pkg_share, "rviz", "FootstepPlanner.rviz")

    config_path = PathJoinSubstitution([
        get_package_share_directory("baseline_footstep_planner"), "config", "FootstepPlanner.yaml"
    ])
    # config = 
    return launch.LaunchDescription([
        # Load parameters from YAML file
        DeclareLaunchArgument(
            "config_file",
            default_value=config_path,
            description="Path to config file"
        ),

        Node(
            package="baseline_footstep_planner",
            executable="FootstepPlannerNode",
            name="footstep_planner",
            output="screen",
            parameters=[LaunchConfiguration("config_file")],
        ),

        Node(
            package="baseline_footstep_planner",
            executable="FootstepPlannerRvizServer.py",
            name="footstep_planner_rviz_server",
            output="screen",
            parameters=[LaunchConfiguration("config_file")],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file]
        ),
    ])

