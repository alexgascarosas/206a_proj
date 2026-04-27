#!/usr/bin/env python3
"""
ball_balance_controller/mock_ball_publisher.py

Publishes synthetic BallState messages for offline PID testing.
The ball traces a circle of configurable radius and period.
Velocity is the analytical derivative — no noise added by default.

Parameters
──────────
  radius_mm   float  Radius of the circle  (default 10.0 mm)
  period_s    float  Period of one lap      (default 8.0 s)
  publish_hz  float  Publish rate           (default 30.0 Hz)
  add_noise   bool   Add Gaussian noise     (default false)
  noise_pos   float  Position noise std-dev (default 1.0 mm)
  noise_vel   float  Velocity noise std-dev (default 5.0 mm/s)
"""

import time
import math

import rclpy
from rclpy.node import Node

from ball_tracker_msgs.msg import BallState


class MockBallPublisher(Node):
    def __init__(self):
        super().__init__("mock_ball_publisher")

        self.declare_parameter("radius_mm",  10.0)
        self.declare_parameter("period_s",    8.0)
        self.declare_parameter("publish_hz", 30.0)
        self.declare_parameter("add_noise",  False)
        self.declare_parameter("noise_pos",   1.0)
        self.declare_parameter("noise_vel",   5.0)

        self.pub = self.create_publisher(BallState, "/ball_state", 10)
        self.t0  = time.time()

        hz = self.get_parameter("publish_hz").value
        self.create_timer(1.0 / hz, self._publish)
        self.get_logger().info(
            f"MockBallPublisher: r={self.get_parameter('radius_mm').value} mm, "
            f"T={self.get_parameter('period_s').value} s, "
            f"rate={hz} Hz"
        )

    def _publish(self):
        import numpy as np

        t     = time.time() - self.t0
        R     = self.get_parameter("radius_mm").value
        T     = self.get_parameter("period_s").value
        omega = 2.0 * math.pi / T

        x  =  R * math.cos(omega * t)
        y  =  R * math.sin(omega * t)
        vx = -R * omega * math.sin(omega * t)
        vy =  R * omega * math.cos(omega * t)

        if self.get_parameter("add_noise").value:
            sp = self.get_parameter("noise_pos").value
            sv = self.get_parameter("noise_vel").value
            x  += np.random.normal(0, sp)
            y  += np.random.normal(0, sp)
            vx += np.random.normal(0, sv)
            vy += np.random.normal(0, sv)

        msg = BallState()
        msg.ball_found    = True
        msg.markers_found = 4
        msg.x  = float(x)
        msg.y  = float(y)
        msg.vx = float(vx)
        msg.vy = float(vy)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockBallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
