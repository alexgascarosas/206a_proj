"""
test/test_controller_math.py

Pure-Python unit tests for PID math and IK helper functions.
These do NOT require a running ROS2 environment.

Run with:
  pytest test/test_controller_math.py -v
"""

import math
import numpy as np
import pytest


# ─────────────────────────────────────────────────────────────────────────────
# Helpers extracted from controller (tested in isolation)
# ─────────────────────────────────────────────────────────────────────────────

def rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]], dtype=float)


def build_target_transform(
    pitch_cmd: float,
    roll_cmd: float,
    tcp_pos: np.ndarray,
    base_ori: np.ndarray,
) -> np.ndarray:
    tilt = rot_x(pitch_cmd) @ rot_y(roll_cmd)
    R    = base_ori @ tilt
    T    = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = tcp_pos
    return T


def compute_pid(
    x_mm, y_mm, vx, vy,
    x_ref, y_ref, vx_ref, vy_ref,
    int_x, int_y, dt,
    kp=0.0012, kd=0.0004, ki=0.0,
    deadband=1.0, integral_limit=80.0, velocity_clip=120.0,
    roll_sign=1.0, pitch_sign=1.0, max_tilt=0.06,
):
    ex = x_mm - x_ref
    ey = y_mm - y_ref
    if abs(ex) < deadband: ex = 0.0
    if abs(ey) < deadband: ey = 0.0

    evx = float(np.clip(vx - vx_ref, -velocity_clip, velocity_clip))
    evy = float(np.clip(vy - vy_ref, -velocity_clip, velocity_clip))

    int_x = float(np.clip(int_x + ex * dt, -integral_limit, integral_limit))
    int_y = float(np.clip(int_y + ey * dt, -integral_limit, integral_limit))

    ux = kp * ex + kd * evx + ki * int_x
    uy = kp * ey + kd * evy + ki * int_y

    roll  = float(np.clip(-roll_sign  * uy, -max_tilt, max_tilt))
    pitch = float(np.clip( pitch_sign * ux, -max_tilt, max_tilt))
    return pitch, roll, int_x, int_y


# ─────────────────────────────────────────────────────────────────────────────
# Rotation matrix tests
# ─────────────────────────────────────────────────────────────────────────────

class TestRotationMatrices:

    def test_rot_x_zero(self):
        assert np.allclose(rot_x(0.0), np.eye(3))

    def test_rot_y_zero(self):
        assert np.allclose(rot_y(0.0), np.eye(3))

    def test_rot_x_90(self):
        R = rot_x(math.pi / 2)
        v = R @ np.array([0, 1, 0])
        assert np.allclose(v, [0, 0, 1], atol=1e-10)

    def test_rot_y_90(self):
        R = rot_y(math.pi / 2)
        v = R @ np.array([1, 0, 0])
        assert np.allclose(v, [0, 0, -1], atol=1e-10)

    def test_rot_x_orthogonal(self):
        R = rot_x(0.3)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)

    def test_rot_y_orthogonal(self):
        R = rot_y(0.3)
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-12)

    def test_composition_does_not_commute(self):
        Rx = rot_x(0.2)
        Ry = rot_y(0.3)
        assert not np.allclose(Rx @ Ry, Ry @ Rx)


# ─────────────────────────────────────────────────────────────────────────────
# Transform builder
# ─────────────────────────────────────────────────────────────────────────────

class TestTargetTransform:
    tcp   = np.array([0.3, 0.0, 0.5])
    base_ori = np.eye(3)

    def test_zero_tilt_preserves_base(self):
        T = build_target_transform(0.0, 0.0, self.tcp, self.base_ori)
        assert np.allclose(T[:3, :3], np.eye(3))
        assert np.allclose(T[:3, 3],  self.tcp)

    def test_position_always_fixed(self):
        for p, r in [(0.05, 0.0), (0.0, 0.05), (0.03, -0.03)]:
            T = build_target_transform(p, r, self.tcp, self.base_ori)
            assert np.allclose(T[:3, 3], self.tcp), f"Position changed for pitch={p} roll={r}"

    def test_result_is_valid_rotation(self):
        T = build_target_transform(0.04, -0.02, self.tcp, self.base_ori)
        R = T[:3, :3]
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-10)
        assert np.isclose(np.linalg.det(R), 1.0, atol=1e-10)

    def test_pitch_tilts_x_axis(self):
        T = build_target_transform(0.05, 0.0, self.tcp, self.base_ori)
        # X column should be unchanged (rotation around X)
        assert np.allclose(T[:3, 0], [1, 0, 0], atol=1e-10)

    def test_roll_tilts_y_axis(self):
        T = build_target_transform(0.0, 0.05, self.tcp, self.base_ori)
        # Y column should be unchanged (rotation around Y)
        assert np.allclose(T[:3, 1], [0, 1, 0], atol=1e-10)


# ─────────────────────────────────────────────────────────────────────────────
# PID tests
# ─────────────────────────────────────────────────────────────────────────────

class TestPID:

    def test_zero_error_zero_output(self):
        pitch, roll, _, _ = compute_pid(
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, dt=0.1,
        )
        assert pitch == 0.0
        assert roll  == 0.0

    def test_positive_x_error_positive_pitch(self):
        pitch, _, _, _ = compute_pid(
            20.0, 0.0, 0.0, 0.0,
            0.0,  0.0, 0.0, 0.0,
            0.0, 0.0, dt=0.1,
        )
        assert pitch > 0.0

    def test_positive_y_error_negative_roll(self):
        _, roll, _, _ = compute_pid(
            0.0, 20.0, 0.0, 0.0,
            0.0, 0.0,  0.0, 0.0,
            0.0, 0.0, dt=0.1,
        )
        assert roll < 0.0  # roll_sign=1 → roll = -uy, uy>0 → roll<0

    def test_deadband_suppresses_small_error(self):
        pitch, roll, _, _ = compute_pid(
            0.5, 0.5, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, dt=0.1, deadband=1.0,
        )
        assert pitch == 0.0
        assert roll  == 0.0

    def test_output_clipped_to_max_tilt(self):
        # Very large error → should saturate at max_tilt
        pitch, roll, _, _ = compute_pid(
            5000.0, 5000.0, 0.0, 0.0,
            0.0,    0.0,    0.0, 0.0,
            0.0, 0.0, dt=0.1, max_tilt=0.06,
        )
        assert abs(pitch) <= 0.06 + 1e-9
        assert abs(roll)  <= 0.06 + 1e-9

    def test_integral_accumulates(self):
        _, _, int_x, int_y = compute_pid(
            10.0, 10.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0,
            0.0, 0.0, dt=0.1, ki=0.001,
        )
        assert int_x > 0.0
        assert int_y > 0.0

    def test_integral_clamped(self):
        big_int = 1e9
        _, _, int_x, _ = compute_pid(
            10.0, 0.0, 0.0, 0.0,
            0.0,  0.0, 0.0, 0.0,
            big_int, 0.0, dt=0.1, ki=0.001, integral_limit=80.0,
        )
        assert int_x <= 80.0

    def test_velocity_error_contributes(self):
        # Same position error, but one has high velocity error
        pitch_no_vel, _, _, _ = compute_pid(
            10.0, 0.0,  0.0, 0.0,
            0.0,  0.0,  0.0, 0.0,
            0.0, 0.0, dt=0.1,
        )
        pitch_high_vel, _, _, _ = compute_pid(
            10.0, 0.0, 50.0, 0.0,
            0.0,  0.0,  0.0, 0.0,
            0.0, 0.0, dt=0.1,
        )
        # Higher velocity should increase pitch (more damping)
        assert pitch_high_vel > pitch_no_vel

    def test_sign_flip_reverses_direction(self):
        pitch_pos, _, _, _ = compute_pid(
            10.0, 0.0, 0.0, 0.0,
            0.0,  0.0, 0.0, 0.0,
            0.0, 0.0, dt=0.1, pitch_sign=1.0,
        )
        pitch_neg, _, _, _ = compute_pid(
            10.0, 0.0, 0.0, 0.0,
            0.0,  0.0, 0.0, 0.0,
            0.0, 0.0, dt=0.1, pitch_sign=-1.0,
        )
        assert pitch_pos > 0.0
        assert pitch_neg < 0.0


# ─────────────────────────────────────────────────────────────────────────────
# ikpy format conversion tests (no robot required)
# ─────────────────────────────────────────────────────────────────────────────

class TestIkpyConversion:

    def _ros_to_ikpy(self, ros: np.ndarray) -> np.ndarray:
        return np.concatenate([[0.0], ros, [0.0]])

    def _ikpy_to_ros(self, ikpy: np.ndarray) -> np.ndarray:
        return np.array(ikpy[1:7], dtype=float)

    def test_round_trip(self):
        ros = np.array([0.5, -2.35, 2.65, -1.85, 1.57, 0.0])
        assert np.allclose(self._ikpy_to_ros(self._ros_to_ikpy(ros)), ros)

    def test_ikpy_length(self):
        ros = np.zeros(6)
        ikpy = self._ros_to_ikpy(ros)
        assert len(ikpy) == 8   # 1 base + 6 joints + 1 end

    def test_padding_is_zero(self):
        ros  = np.ones(6)
        ikpy = self._ros_to_ikpy(ros)
        assert ikpy[0] == 0.0
        assert ikpy[7] == 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Reference trajectory tests
# ─────────────────────────────────────────────────────────────────────────────

class TestReference:

    def _circle(self, t, R=20.0, T=20.0):
        omega = 2.0 * math.pi / T
        return (
             R * math.cos(omega * t),
             R * math.sin(omega * t),
            -R * omega * math.sin(omega * t),
             R * omega * math.cos(omega * t),
        )

    def test_circle_radius_correct(self):
        for t in np.linspace(0, 20, 50):
            x, y, _, _ = self._circle(t, R=20.0)
            assert abs(math.hypot(x, y) - 20.0) < 1e-9

    def test_circle_velocity_tangent(self):
        """Velocity should be perpendicular to position vector."""
        for t in np.linspace(0.1, 19.9, 20):
            x, y, vx, vy = self._circle(t, R=20.0)
            dot = x * vx + y * vy
            assert abs(dot) < 1e-9, f"Velocity not tangent at t={t}"

    def test_circle_at_zero_starts_on_x_axis(self):
        x, y, _, _ = self._circle(0.0, R=20.0)
        assert math.isclose(x, 20.0)
        assert math.isclose(y, 0.0)
