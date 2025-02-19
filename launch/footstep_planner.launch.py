import os
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    pkg_share = FindPackageShare("baseline_footstep_planner").find("baseline_footstep_planner")
    rviz_config_file = os.path.join(pkg_share, "rviz", "FootstepPlanner.rviz")
    config_file = LaunchConfiguration("config_file")

    config_foot_step_planner = PathJoinSubstitution(
        [
            get_package_share_directory('baseline_footstep_planner'),
            'config',
            config_file
        ]
    )
    # config = 
    return launch.LaunchDescription([
        # Load parameters from YAML file
        DeclareLaunchArgument("config_file", default_value="FootstepPlanner.yaml", description="Path to config file"),

        Node(
            package="baseline_footstep_planner",
            executable="FootstepPlannerNode",
            name="footstep_planner",
            output="screen",
            parameters=[
                config_foot_step_planner,
                {
                    "theta_divide_num": 64,
                    "xy_divide_step": 0.01, # [m]
                    "cost_scale": 1.0e3,
                    "cost_theta_scale": 0.0, # [m/rad]
                    "step_cost": 1.0,
                    "heuristic_type": "DijkstraPath",
                    "dijkstra_path_heuristic_expand_scale": 5.0,
                    "nominal_foot_separation": 0.2, # [m]
                    "r2l_action_cont_list": "[[0.2, 0.0, 0.0],[-0.1, 0.0, 0.0],[0.0, 0.1, 0.0],[0.0, -0.05, 0.0],[0.0, 0.0, 0.349],[0.0, 0,0, -0.349]]", # [m], [rad]
                    "r2l_reachable_min": [-0.05, 0.0, -0.1745], # [m], [rad]
                    "r2l_reachable_max": [0.1, 0.05, 0.1745],
                    "rect_obstacle_list": "[[1.0, 0.0, 0.2, 1.0], [0.0, 1.0, 1.0, 0.2]]",
                    "start_pose": [0.0, 0.0, 0.0],
                    "goal_pose": [2.0, 1.5, 0.0],
                }
            ]
        ),

        Node(
            package="baseline_footstep_planner",
            executable="FootstepPlannerRvizServer.py",
            name="footstep_planner_rviz_server",
            output="screen",
            parameters=[{
                "nominal_foot_separation": 0.2, # [m]
                "rect_obstacle_list": "[[1.0, 0.0, 0.2, 1.0], [0.0, 1.0, 1.0, 0.2]]"
            }]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", rviz_config_file]
        ),
    ])

