#!/usr/bin/env python3
"""
ball_balance_controller/controller.py

UR7e Ball-on-Plate PID Controller with ikpy IK backend.

Architecture
------------
  /ball_state  (BallState)
       │
       ▼
  PID + reference generator
       │
       ▼  pitch_cmd, roll_cmd  (rad)
  _build_target_transform()   ← fixed TCP position from FK at startup
       │
       ▼  4×4 homogeneous transform
  ikpy inverse_kinematics()
       │
       ▼  all 6 joint angles
  rate-limit + low-pass filter
       │
       ▼
  FollowJointTrajectory action → /scaled_joint_trajectory_controller

Safety layers (in order of priority)
--------------------------------------
  1. Hard edge rescue  – ball > edge_limit  → max tilt
  2. Joint rate limit  – clip Δjoint per step
  3. Wrist tilt limit  – clip pitch/roll commands
  4. Stale tracker     – smooth return to flat
  5. Missing ball      – smooth return to flat
  6. IK failure        – skip step, log warning
"""

import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration as DurationMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from ball_tracker_msgs.msg import BallState

try:
    import ikpy.chain
except ImportError:
    raise ImportError(
        "ikpy not installed. Run:  pip install ikpy"
    )


class BallBalanceController(Node):
    """ROS2 node: PID ball-on-plate controller for UR7e using full IK."""

    # ------------------------------------------------------------------ init --

    def __init__(self):
        super().__init__("ball_balance_controller")

        self._declare_parameters()
        self._init_ik_chain()
        self._init_state()
        self._init_ros()

        self.get_logger().info("BallBalanceController ready.")

    # --------------------------------------------------------- parameter setup --

    def _declare_parameters(self):
        self.declare_parameter("urdf_path", "")

        # Base pose (6 joints, radians)
        self.declare_parameter(
            "base_joints",
            [0.5, -2.35, 2.65, -1.85, 1.57, 0.0],
        )

        # PID gains
        self.declare_parameter("kp_x", 0.00120)
        self.declare_parameter("kd_x", 0.00040)
        self.declare_parameter("ki_x", 0.00000)
        self.declare_parameter("kp_y", 0.00120)
        self.declare_parameter("kd_y", 0.00040)
        self.declare_parameter("ki_y", 0.00000)

        # Sign flips (set -1.0 if ball moves away from centre after command)
        self.declare_parameter("roll_sign",  1.0)
        self.declare_parameter("pitch_sign", 1.0)

        # Safety
        self.declare_parameter("max_tilt_rad",         0.060)
        self.declare_parameter("max_joint_delta_rad",  0.15)
        self.declare_parameter("edge_limit_x_mm",      60.0)
        self.declare_parameter("edge_limit_y_mm",      60.0)
        self.declare_parameter("deadband_mm",          1.0)
        self.declare_parameter("integral_limit",       80.0)
        self.declare_parameter("velocity_clip",        120.0)

        # Timing / filtering
        self.declare_parameter("command_alpha",        0.70)
        self.declare_parameter("command_rate_hz",      10.0)
        self.declare_parameter("trajectory_time",      0.15)
        self.declare_parameter("return_to_base_rate",  0.25)

        # Reference trajectory
        self.declare_parameter("track_circle",         True)
        self.declare_parameter("track_figure8",        False)
        self.declare_parameter("circle_radius_mm",     20.0)
        self.declare_parameter("circle_period_s",      20.0)
        self.declare_parameter("settle_time_s",        0.0)

        # Lookahead (currently informational; extend for predictive control)
        self.declare_parameter("lookahead_s",          0.35)

        self._p = lambda name: self.get_parameter(name).value

    # --------------------------------------------------------------- IK chain --

    def _init_ik_chain(self):
        urdf_path = self._p("urdf_path")
        if not urdf_path:
            raise RuntimeError(
                "Parameter 'urdf_path' is not set. "
                "Pass it via launch file or:  --ros-args -p urdf_path:=/path/to/ur7e.urdf"
            )

        self.get_logger().info(f"Loading IK chain from: {urdf_path}")
        # active_links_mask length must equal number of links in the chain.
        # Standard UR7e URDF: base_link(fixed) + 6 revolute + tool0(fixed) = 8 links
        # → mask = [False, True, True, True, True, True, True, False]
        # If your URDF differs, adjust accordingly (check len(chain.links)).
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=["base_link"],
            active_links_mask=[False, True, True, True, True, True, True, False],
        )
        self.get_logger().info(
            f"IK chain loaded: {len(self.chain.links)} links, "
            f"active: {sum(self.chain.active_links_mask)}"
        )

        # FK at base pose → fix TCP position for all future IK calls
        base_joints_ros = np.array(self._p("base_joints"), dtype=float)
        base_ikpy = self._ros_to_ikpy(base_joints_ros)
        base_fk   = self.chain.forward_kinematics(base_ikpy)
        self.tcp_position    = base_fk[:3, 3].copy()
        self.base_orientation = base_fk[:3, :3].copy()

        self.get_logger().info(
            f"TCP fixed position (m): {np.round(self.tcp_position, 4).tolist()}"
        )

    # ------------------------------------------------------------------ state --

    def _init_state(self):
        self.base_joints = np.array(self._p("base_joints"), dtype=float)

        # PID integrators
        self.int_x = 0.0
        self.int_y = 0.0

        # Filtered joint state (both formats kept in sync)
        self.filtered_joints_ros  = self.base_joints.copy()
        self.filtered_joints_ikpy = self._ros_to_ikpy(self.filtered_joints_ros)

        # Tracker message cache
        self.last_msg:      Optional[BallState] = None
        self.last_msg_time: Optional[float]     = None

        # Timing
        self.last_control_time = time.time()
        self.last_command_time = 0.0
        self.start_time        = time.time()

        self.last_goal_handle = None

    # ----------------------------------------------------------------- ROS IO --

    def _init_ros(self):
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

        self.sub_ball = self.create_subscription(
            BallState,
            "/ball_state",
            self._ball_callback,
            10,
        )

        self.get_logger().info("Waiting for trajectory action server …")
        if not self.traj_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Trajectory action server not available after 10 s.")

        self.get_logger().info("Moving to base pose …")
        self._send_joint_goal(self.base_joints.tolist(), trajectory_time=2.0)

        period = 1.0 / self._p("command_rate_hz")
        self.control_timer = self.create_timer(period, self._control_step)

    # ============================================================= CALLBACKS ==

    def _ball_callback(self, msg: BallState):
        self.last_msg      = msg
        self.last_msg_time = time.time()

    # ============================================================== REFERENCE ==

    def _get_reference(self):
        """Return (x_ref, y_ref, vx_ref, vy_ref) in mm / mm·s⁻¹."""
        t = time.time() - self.start_time
        if t < self._p("settle_time_s"):
            return 0.0, 0.0, 0.0, 0.0

        t_eff  = t - self._p("settle_time_s")
        R      = self._p("circle_radius_mm")
        omega  = 2.0 * np.pi / self._p("circle_period_s")

        if self._p("track_circle"):
            return (
                 R * np.cos(omega * t_eff),
                 R * np.sin(omega * t_eff),
                -R * omega * np.sin(omega * t_eff),
                 R * omega * np.cos(omega * t_eff),
            )

        if self._p("track_figure8"):
            return (
                R       * np.sin(omega * t_eff),
                0.5 * R * np.sin(2.0 * omega * t_eff),
                R       * omega * np.cos(omega * t_eff),
                R       * omega * np.cos(2.0 * omega * t_eff),
            )

        return 0.0, 0.0, 0.0, 0.0

    # ================================================================== PID ===

    def _compute_control(
        self,
        x_mm: float,
        y_mm: float,
        vx_mm_s: float,
        vy_mm_s: float,
        dt: float,
    ) -> dict:
        x_ref, y_ref, vx_ref, vy_ref = self._get_reference()

        # Position errors
        ex = x_mm - x_ref
        ey = y_mm - y_ref
        if abs(ex) < self._p("deadband_mm"):
            ex = 0.0
        if abs(ey) < self._p("deadband_mm"):
            ey = 0.0

        # Velocity errors (clipped for robustness)
        vc = self._p("velocity_clip")
        evx = float(np.clip(vx_mm_s - vx_ref, -vc, vc))
        evy = float(np.clip(vy_mm_s - vy_ref, -vc, vc))

        # Integrators
        il = self._p("integral_limit")
        self.int_x = float(np.clip(self.int_x + ex * dt, -il, il))
        self.int_y = float(np.clip(self.int_y + ey * dt, -il, il))

        # PID outputs
        ux = (self._p("kp_x") * ex
              + self._p("kd_x") * evx
              + self._p("ki_x") * self.int_x)
        uy = (self._p("kp_y") * ey
              + self._p("kd_y") * evy
              + self._p("ki_y") * self.int_y)

        # Hard edge rescue: override PID with max tilt
        if abs(x_mm) > self._p("edge_limit_x_mm"):
            ux = np.sign(x_mm) * 999.0
        if abs(y_mm) > self._p("edge_limit_y_mm"):
            uy = np.sign(y_mm) * 999.0

        mt = self._p("max_tilt_rad")
        roll_cmd  = float(np.clip(-self._p("roll_sign")  * uy, -mt, mt))
        pitch_cmd = float(np.clip( self._p("pitch_sign") * ux, -mt, mt))

        return {
            "x_ref": x_ref, "y_ref": y_ref,
            "ex": ex,       "ey": ey,
            "evx": evx,     "evy": evy,
            "ux": ux,       "uy": uy,
            "roll_cmd":  roll_cmd,
            "pitch_cmd": pitch_cmd,
        }

    # ================================================================= IK ====

    def _ros_to_ikpy(self, ros_joints: np.ndarray) -> np.ndarray:
        """Pad 6-element ROS array → ikpy chain-length array."""
        return np.concatenate([[0.0], ros_joints, [0.0]])

    def _ikpy_to_ros(self, ikpy_joints: np.ndarray) -> np.ndarray:
        """Strip padding → 6 ROS joint values."""
        return np.array(ikpy_joints[1:7], dtype=float)

    def _rot_x(self, a: float) -> np.ndarray:
        c, s = np.cos(a), np.sin(a)
        return np.array([[1, 0,  0],
                         [0, c, -s],
                         [0, s,  c]], dtype=float)

    def _rot_y(self, a: float) -> np.ndarray:
        c, s = np.cos(a), np.sin(a)
        return np.array([[ c, 0, s],
                         [ 0, 1, 0],
                         [-s, 0, c]], dtype=float)

    def _build_target_transform(self, pitch_cmd: float, roll_cmd: float) -> np.ndarray:
        """
        4×4 homogeneous target for IK.
        Position  : self.tcp_position  (fixed — plate stays in place)
        Orientation: base_orientation @ Rx(pitch) @ Ry(roll)
        """
        tilt   = self._rot_x(pitch_cmd) @ self._rot_y(roll_cmd)
        target_rot = self.base_orientation @ tilt

        T = np.eye(4)
        T[:3, :3] = target_rot
        T[:3,  3] = self.tcp_position
        return T

    def _solve_ik(self, T: np.ndarray) -> Optional[np.ndarray]:
        """
        Run ikpy IK.
        Returns ikpy-format joint array, or None on failure.
        Warm-started from the previous filtered solution for fast convergence.
        """
        try:
            sol = self.chain.inverse_kinematics(
                target_position=T[:3, 3],
                target_orientation=T[:3, :3],
                orientation_mode="all",
                initial_position=self.filtered_joints_ikpy,
            )
            return sol
        except Exception as exc:
            self.get_logger().error(f"IK solver error: {exc}")
            return None

    # ============================================================ CONTROL STEP ==

    def _control_step(self):
        now = time.time()

        period = 1.0 / self._p("command_rate_hz")
        if now - self.last_command_time < period:
            return

        # ── guard: no data yet ────────────────────────────────────────────────
        if self.last_msg is None:
            return

        msg_age = (now - self.last_msg_time) if self.last_msg_time else 999.0

        # ── guard: stale tracker ──────────────────────────────────────────────
        if msg_age > 0.5:
            self.get_logger().warn("BallState stale (>0.5 s). Returning to base.")
            self._return_to_base()
            self.last_command_time = now
            return

        msg = self.last_msg

        # ── guard: bad marker visibility ──────────────────────────────────────
        if msg.markers_found < 3:
            self.get_logger().warn(
                f"Markers found: {msg.markers_found}/4. Returning to base."
            )
            self._return_to_base()
            self.last_command_time = now
            return

        # ── guard: ball not visible (allow brief Kalman prediction) ───────────
        if not msg.ball_found and msg_age > 0.25:
            self.get_logger().warn("Ball not found. Returning to base.")
            self._return_to_base()
            self.last_command_time = now
            return

        # ── PID ───────────────────────────────────────────────────────────────
        dt = max(now - self.last_control_time, 1e-3)
        self.last_control_time = now

        ctl = self._compute_control(
            float(msg.x), float(msg.y),
            float(msg.vx), float(msg.vy),
            dt,
        )

        # ── IK ────────────────────────────────────────────────────────────────
        T_target = self._build_target_transform(ctl["pitch_cmd"], ctl["roll_cmd"])
        sol_ikpy  = self._solve_ik(T_target)

        if sol_ikpy is None:
            self.get_logger().warn("IK failed — skipping step.")
            self.last_command_time = now
            return

        raw_ros = self._ikpy_to_ros(sol_ikpy)

        # ── Joint rate limiting ───────────────────────────────────────────────
        max_delta = self._p("max_joint_delta_rad")
        delta     = np.clip(
            raw_ros - self.filtered_joints_ros,
            -max_delta, max_delta,
        )
        raw_ros = self.filtered_joints_ros + delta

        # ── Low-pass filter (all 6 joints) ────────────────────────────────────
        alpha = self._p("command_alpha")
        self.filtered_joints_ros = (
            (1.0 - alpha) * self.filtered_joints_ros + alpha * raw_ros
        )
        self.filtered_joints_ikpy = self._ros_to_ikpy(self.filtered_joints_ros)

        # ── Send goal ─────────────────────────────────────────────────────────
        self._send_joint_goal(
            self.filtered_joints_ros.tolist(),
            trajectory_time=self._p("trajectory_time"),
        )
        self.last_command_time = now

        self.get_logger().info(
            f"x={msg.x:+7.1f}mm y={msg.y:+7.1f}mm  "
            f"ref=({ctl['x_ref']:+6.1f},{ctl['y_ref']:+6.1f})  "
            f"vx={msg.vx:+7.1f} vy={msg.vy:+7.1f}  "
            f"pitch={ctl['pitch_cmd']:+.4f}r roll={ctl['roll_cmd']:+.4f}r  "
            f"j={np.round(self.filtered_joints_ros,3).tolist()}"
        )

    # ============================================================ TRAJECTORY ==

    def _send_joint_goal(
        self,
        joint_positions: List[float],
        trajectory_time: float = 0.15,
    ):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions     = joint_positions
        pt.velocities    = [0.0] * len(joint_positions)
        pt.time_from_start = DurationMsg(
            sec=int(trajectory_time),
            nanosec=int((trajectory_time % 1.0) * 1e9),
        )
        traj.points.append(pt)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory          = traj
        goal_msg.goal_time_tolerance = DurationMsg(sec=0, nanosec=int(2e8))

        future = self.traj_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            gh = future.result()
            if gh is None or not gh.accepted:
                self.get_logger().warn("Trajectory goal not accepted.")
                return
            self.last_goal_handle = gh
        except Exception as exc:
            self.get_logger().error(f"Goal send error: {exc}")

    # ============================================================ BASE RETURN ==

    def _return_to_base(self):
        """Exponential blend toward flat pose; decay integrators."""
        self.int_x *= 0.8
        self.int_y *= 0.8

        rate = self._p("return_to_base_rate")
        self.filtered_joints_ros = (
            (1.0 - rate) * self.filtered_joints_ros + rate * self.base_joints
        )
        self.filtered_joints_ikpy = self._ros_to_ikpy(self.filtered_joints_ros)

        self._send_joint_goal(
            self.filtered_joints_ros.tolist(),
            trajectory_time=self._p("trajectory_time"),
        )


# ================================================================= ENTRY POINT ==

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BallBalanceController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
