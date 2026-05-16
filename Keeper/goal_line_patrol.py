#!/usr/bin/env python3
"""Goal-line patrol: dog moves left-right along the goal line using VRPN motion capture."""

import math
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from protocol.msg import MotionServoCmd


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_angle(rad):
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class GoalLinePatrol(Node):
    def __init__(self):
        super().__init__("goal_line_patrol")

        # ---- Configurable parameters ----
        self.declare_parameter("ball_tracker", "ball")
        self.declare_parameter("dog_tracker", "LYT")
        self.declare_parameter("goal_tracker", "goal_right")
        self.declare_parameter("patrol_speed", 0.2)
        self.declare_parameter("patrol_range", 1.0)
        self.declare_parameter("y_threshold", 0.1)
        self.declare_parameter("yaw_gain", 0.5)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("face_ball", True)

        ball_tracker = self.get_parameter("ball_tracker").value
        dog_tracker = self.get_parameter("dog_tracker").value
        goal_tracker = self.get_parameter("goal_tracker").value

        # ---- VRPN subscriptions ----
        self.ball_xy = None
        self.goal_xy = None
        self.dog_xy = None
        self.dog_yaw = None
        self.ball_t = 0.0
        self.goal_t = 0.0
        self.dog_t = 0.0

        self.create_subscription(
            PoseStamped, f"/vrpn/{ball_tracker}/pose", self._on_ball, 10
        )
        self.create_subscription(
            PoseStamped, f"/vrpn/{goal_tracker}/pose", self._on_goal, 10
        )
        self.create_subscription(
            PoseStamped, f"/vrpn/{dog_tracker}/pose", self._on_dog, 10
        )

        # ---- Motion command publisher ----
        ns = self._get_namespace()
        self.cmd_pub = self.create_publisher(
            MotionServoCmd, f"/{ns}/motion_servo_cmd", 10
        )

        # ---- Patrol state ----
        self.patrol_dir = 1  # +1 = move right, -1 = move left

        # ---- Control loop at 10 Hz ----
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Goal-line patrol node started.")

    def _get_namespace(self):
        import socket
        import re

        hostname = socket.getfqdn(socket.gethostname())
        try:
            with open("/sys/class/net/eth0/address") as f:
                mac = f.read().strip().replace(":", "")
        except Exception:
            mac = "00_00_00_00_00_00"
        namespace = hostname + "_" + mac
        return re.sub("[^0-9a-zA-Z]+", "_", namespace)

    # ---- VRPN callbacks ----
    def _on_ball(self, msg):
        self.ball_xy = (msg.pose.position.x, msg.pose.position.y)
        self.ball_t = time.monotonic()

    def _on_goal(self, msg):
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.goal_t = time.monotonic()

    def _on_dog(self, msg):
        self.dog_xy = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        # Yaw from quaternion (rotation about Z axis)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.dog_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.dog_t = time.monotonic()

    # ---- Main control loop ----
    def control_loop(self):
        now = time.monotonic()

        # Check data freshness
        if self.dog_xy is None or self.dog_yaw is None:
            self.get_logger().warn("Waiting for dog VRPN data...")
            return
        if self.goal_xy is None:
            self.get_logger().warn("Waiting for goal VRPN data...")
            return

        if now - self.dog_t > 2.0:
            self.get_logger().warn("Dog data stale, stopping.")
            self._send_cmd(0.0, 0.0, 0.0)
            return

        dog_x, dog_y = self.dog_xy
        dog_yaw = self.dog_yaw
        goal_x, goal_y = self.goal_xy

        patrol_speed = self.get_parameter("patrol_speed").value
        patrol_range = self.get_parameter("patrol_range").value
        y_threshold = self.get_parameter("y_threshold").value

        # Patrol along the goal line (perpendicular to the goal-to-field direction)
        # The goal line is perpendicular to the vector from goal center to field center.
        # Here we simply patrol along the Y-axis relative to the goal position.

        # Calculate distance from dog to goal along the patrol axis (Y)
        lateral_offset = dog_y - goal_y

        # Reverse direction when reaching patrol range limits
        if lateral_offset > patrol_range:
            self.patrol_dir = -1
        elif lateral_offset < -patrol_range:
            self.patrol_dir = 1

        # Lateral velocity command (in body frame, vy = lateral movement)
        vy = self.patrol_dir * patrol_speed

        # If ball data is available, face the ball; otherwise face forward
        vyaw = 0.0
        if self.ball_xy is not None and now - self.ball_t < 2.0:
            ball_x, ball_y = self.ball_xy
            desired_yaw = math.atan2(ball_y - dog_y, ball_x - dog_x)
            yaw_error = norm_angle(desired_yaw - dog_yaw)
            yaw_gain = self.get_parameter("yaw_gain").value
            max_yaw_rate = self.get_parameter("max_yaw_rate").value
            vyaw = clamp(yaw_gain * yaw_error, -max_yaw_rate, max_yaw_rate)

        self._send_cmd(0.0, vy, vyaw)

        self.get_logger().info(
            f"dog=({dog_x:.2f},{dog_y:.2f}) goal=({goal_x:.2f},{goal_y:.2f}) "
            f"lateral={lateral_offset:.2f} dir={self.patrol_dir} vy={vy:.2f} vyaw={vyaw:.2f}"
        )

    def _send_cmd(self, vx, vy, vyaw):
        cmd = MotionServoCmd()
        cmd.motion_id = 303
        cmd.cmd_type = 1
        cmd.value = 2
        cmd.vel_des = [float(vx), float(vy), float(vyaw)]
        cmd.step_height = [0.05, 0.05]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalLinePatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
