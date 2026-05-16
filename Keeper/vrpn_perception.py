#!/usr/bin/env python3
"""VRPN perception: subscriptions, ball velocity, intercept prediction, helpers."""

import math
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# Gait IDs
SLOW_TROT = 303
FAST_TROT = 305
MEDIUM_TROT = 308

CMD_DATA = 1
GAIT_STANDARD = 2


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def norm_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class VrpnPerception:
    """VRPN subscriptions + ball velocity estimation + intercept prediction."""

    def __init__(
        self,
        node: Node,
        ball_tracker: str,
        dog_tracker: str,
        goal_tracker: str,
        yaw_axis: str,
        yaw_alpha: float,
    ) -> None:
        self.node = node
        self.yaw_axis = yaw_axis
        self.yaw_alpha = yaw_alpha

        self.ball_xy: Optional[Tuple[float, float]] = None
        self.goal_xy: Optional[Tuple[float, float]] = None
        self.dog_xy: Optional[Tuple[float, float]] = None

        self.ball_t: float = 0.0
        self.goal_t: float = 0.0
        self.dog_t: float = 0.0

        self.dog_yaw: Optional[float] = None
        self.yaw_x: float = 0.0
        self.yaw_y: float = 0.0
        self.yaw_z: float = 0.0

        self.ball_vx: float = 0.0
        self.ball_vy: float = 0.0

        node.create_subscription(
            PoseStamped, "/vrpn/{}/pose".format(ball_tracker), self._on_ball, 10
        )
        node.create_subscription(
            PoseStamped, "/vrpn/{}/pose".format(goal_tracker), self._on_goal, 10
        )
        node.create_subscription(
            PoseStamped, "/vrpn/{}/pose".format(dog_tracker), self._on_dog, 10
        )

    def _on_ball(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        x, y = msg.pose.position.x, msg.pose.position.y
        if self.ball_xy is not None and now - self.ball_t > 0.001:
            dt = now - self.ball_t
            raw_vx = (x - self.ball_xy[0]) / dt
            raw_vy = (y - self.ball_xy[1]) / dt
            alpha = 0.3
            self.ball_vx += alpha * (raw_vx - self.ball_vx)
            self.ball_vy += alpha * (raw_vy - self.ball_vy)
        self.ball_xy = (x, y)
        self.ball_t = now

    def predict_intercept_x(self, goal_y: float) -> Optional[float]:
        if self.ball_xy is None:
            return None
        ball_x, ball_y = self.ball_xy
        vy = self.ball_vy
        if vy >= -0.05:
            return None
        dt = (goal_y - ball_y) / vy
        if dt <= 0.0 or dt > 1.5:
            return None
        return ball_x + self.ball_vx * dt

    def _on_goal(self, msg: PoseStamped) -> None:
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.goal_t = time.monotonic()

    def _on_dog(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        self.dog_xy = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        r10 = 2.0 * (q.x * q.y + q.w * q.z)
        r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)
        r01 = 2.0 * (q.x * q.y - q.w * q.z)
        r02 = 2.0 * (q.x * q.z + q.w * q.y)
        r12 = 2.0 * (q.y * q.z - q.w * q.x)
        self.yaw_x = math.atan2(r10, r00)
        self.yaw_y = math.atan2(r11, r01)
        self.yaw_z = math.atan2(r12, r02)
        axis_map = {
            "x": (r00, r10, self.yaw_x),
            "y": (r01, r11, self.yaw_y),
            "z": (r02, r12, self.yaw_z),
        }
        _, _, yaw_raw = axis_map[self.yaw_axis]
        if self.dog_yaw is None:
            self.dog_yaw = yaw_raw
        else:
            d = norm_angle(yaw_raw - self.dog_yaw)
            self.dog_yaw = norm_angle(self.dog_yaw + self.yaw_alpha * d)
        self.dog_t = now

    def ready(self) -> bool:
        return (
            self.ball_xy is not None
            and self.goal_xy is not None
            and self.dog_xy is not None
            and self.dog_yaw is not None
        )

    def stale(self, now: float, max_age: float = 2.0) -> List[str]:
        items: List[str] = []
        if now - self.ball_t > max_age:
            items.append("ball")
        if now - self.goal_t > max_age:
            items.append("goal")
        if now - self.dog_t > max_age:
            items.append("dog")
        return items
