"""
launch/balance_sim.launch.py

Launch the controller together with a mock ball_state publisher so you can
test PID logic without a real camera or robot.

The mock publisher sends a ball circling slowly around the plate centre.
The controller output (joint goals) is echoed to the terminal but the
action server is a no-op stub — no real robot needed.

Usage
─────
  ros2 launch ball_balance_controller balance_sim.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("ball_balance_controller")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([pkg, "config", "pid_params.yaml"]),
        description="Params YAML (urdf_path is ignored in sim mode).",
    )

    # ── Mock ball state publisher ─────────────────────────────────────────────
    # Publishes a BallState with ball circling at 10 mm radius, 8 s period.
    # Replace with your own bag file or custom publisher as needed.
    mock_ball_node = Node(
        package="ball_balance_controller",
        executable="mock_ball_publisher",
        name="mock_ball_publisher",
        output="screen",
        parameters=[
            {
                "radius_mm":  10.0,
                "period_s":    8.0,
                "publish_hz": 30.0,
            }
        ],
    )

    # ── Controller (sim mode: urdf_path left empty → IK skipped gracefully) ──
    # In sim mode the controller will warn "urdf_path not set" and exit.
    # Set a real urdf_path if you want to test IK math offline.
    controller_node = Node(
        package="ball_balance_controller",
        executable="ball_balance_controller",
        name="ball_balance_controller",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription(
        [
            params_file_arg,
            LogInfo(msg="Launching in SIM mode (mock ball publisher + controller) …"),
            mock_ball_node,
            controller_node,
        ]
    )
