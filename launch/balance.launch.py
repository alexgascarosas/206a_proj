"""
launch/balance.launch.py

Primary launch file for the ball-balance PID controller.

Usage
─────
  # Default: loads config/pid_params.yaml, holds centre
  ros2 launch ball_balance_controller balance.launch.py

  # Custom URDF path inline
  ros2 launch ball_balance_controller balance.launch.py \
    urdf_path:=/opt/ros/humble/share/ur_description/urdf/ur7e.urdf

  # Custom params file
  ros2 launch ball_balance_controller balance.launch.py \
    params_file:=/home/user/my_params.yaml

  # Override any parameter inline
  ros2 launch ball_balance_controller balance.launch.py \
    kp_x:=0.0016 kd_x:=0.0006 track_circle:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("ball_balance_controller")

    # ── Launch arguments ──────────────────────────────────────────────────────
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg, "config", "pid_params.yaml"]),
        description="Full path to the ROS2 params YAML file.",
    )

    urdf_path_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value="",
        description=(
            "Absolute path to UR7e URDF. "
            "Overrides the value in the params file when non-empty."
        ),
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level: debug | info | warn | error",
    )

    # ── Node ──────────────────────────────────────────────────────────────────
    controller_node = Node(
        package="ball_balance_controller",
        executable="ball_balance_controller",
        name="ball_balance_controller",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            # Inline override so users can pass urdf_path on the CLI
            {"urdf_path": LaunchConfiguration("urdf_path")},
        ],
        arguments=[
            "--ros-args",
            "--log-level", LaunchConfiguration("log_level"),
        ],
        remappings=[
            # Remap if your ball tracker publishes on a different topic
            ("/ball_state", "/ball_state"),
        ],
    )

    return LaunchDescription(
        [
            params_file_arg,
            urdf_path_arg,
            log_level_arg,
            LogInfo(msg="Launching BallBalanceController (PID + IK) …"),
            controller_node,
        ]
    )
