#!/usr/bin/env python3
"""VRPN perception: subscribes to motion-capture rigid-body poses and produces
fused world-state snapshots including position and velocity."""

import math
import time
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# Gait IDs
SLOW_TROT = 303
FAST_TROT = 305
MEDIUM_TROT = 308


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def norm_angle(rad: float) -> float:
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class MotionCmd:
    def __init__(self, gait: int, vx: float, vy: float, vyaw: float) -> None:
        self.gait = gait
        self.vx = vx
        self.vy = vy
        self.vyaw = vyaw


class PerceptionSnapshot:
    def __init__(
        self,
        t: float,
        dog_x: float,
        dog_y: float,
        dog_yaw: float,
        dog_yaw_x: float,
        dog_yaw_y: float,
        dog_yaw_z: float,
        ball_x: float,
        ball_y: float,
        ball_vx: float,
        ball_vy: float,
        goal_x: float,
        goal_y: float,
        goal_from_tracker: bool,
    ) -> None:
        self.t = t
        self.dog_x = dog_x
        self.dog_y = dog_y
        self.dog_yaw = dog_yaw
        self.dog_yaw_x = dog_yaw_x
        self.dog_yaw_y = dog_yaw_y
        self.dog_yaw_z = dog_yaw_z
        self.ball_x = ball_x
        self.ball_y = ball_y
        self.ball_vx = ball_vx
        self.ball_vy = ball_vy
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_from_tracker = goal_from_tracker


class VrpnPerception:
    """VRPN subscriptions + fused world state with velocity tracking."""

    def __init__(
        self,
        node: Node,
        ball_tracker: str,
        dog_tracker: str,
        goal_tracker: str,
        yaw_axis: str,
        yaw_proj_min: float,
        yaw_alpha: float,
        yaw_offset: float = 0.0,
        vel_smooth: float = 0.3,
    ) -> None:
        self.node = node
        self.yaw_axis = yaw_axis
        self.yaw_proj_min = yaw_proj_min
        self.yaw_alpha = yaw_alpha
        self.yaw_offset = yaw_offset
        self.vel_smooth = vel_smooth

        self.ball_xy: Optional[Tuple[float, float]] = None
        self.goal_xy: Optional[Tuple[float, float]] = None
        self.dog_xy: Optional[Tuple[float, float]] = None

        self.ball_t = 0.0
        self.goal_t = 0.0
        self.dog_t = 0.0

        self.goal_logged = False
        self.last_warn_t = 0.0

        self.dog_yaw: Optional[float] = None
        self.yaw_x: float = 0.0
        self.yaw_y: float = 0.0
        self.yaw_z: float = 0.0

        # Velocity tracking state
        self._ball_vx: float = 0.0
        self._ball_vy: float = 0.0
        self._prev_ball_xy: Optional[Tuple[float, float]] = None
        self._prev_ball_t: float = 0.0

        node.create_subscription(
            PoseStamped,
            "/vrpn/{}/pose".format(ball_tracker),
            self._on_ball,
            10,
        )
        node.create_subscription(
            PoseStamped,
            "/vrpn/{}/pose".format(goal_tracker),
            self._on_goal,
            10,
        )
        node.create_subscription(
            PoseStamped,
            "/vrpn/{}/pose".format(dog_tracker),
            self._on_dog,
            10,
        )

    def _on_ball(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        new_x = msg.pose.position.x
        new_y = msg.pose.position.y

        if self.ball_xy is not None and self._prev_ball_t > 0.0:
            dt = now - self._prev_ball_t
            if dt > 1e-6:
                raw_vx = (new_x - self.ball_xy[0]) / dt
                raw_vy = (new_y - self.ball_xy[1]) / dt
                alpha = min(1.0, dt / self.vel_smooth)
                self._ball_vx += alpha * (raw_vx - self._ball_vx)
                self._ball_vy += alpha * (raw_vy - self._ball_vy)

        self.ball_xy = (new_x, new_y)
        self._prev_ball_xy = self.ball_xy
        self._prev_ball_t = now
        self.ball_t = now

    def _on_goal(self, msg: PoseStamped) -> None:
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.goal_t = time.monotonic()
        if not self.goal_logged:
            self.node.get_logger().info(
                "[GOAL] VRPN goal at x={:.2f} y={:.2f}".format(
                    self.goal_xy[0], self.goal_xy[1]
                )
            )
            self.goal_logged = True

    def _on_dog(self, msg: PoseStamped) -> None:
        now = time.monotonic()
        self.dog_xy = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation

        r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        r01 = 2.0 * (q.x * q.y - q.w * q.z)
        r02 = 2.0 * (q.x * q.z + q.w * q.y)
        r10 = 2.0 * (q.x * q.y + q.w * q.z)
        r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)
        r12 = 2.0 * (q.y * q.z - q.w * q.x)
        r20 = 2.0 * (q.x * q.z - q.w * q.y)
        r21 = 2.0 * (q.y * q.z + q.w * q.x)
        r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)

        self.yaw_x = math.atan2(r10, r00)
        self.yaw_y = math.atan2(r11, r01)
        self.yaw_z = math.atan2(r12, r02)

        axis_map = {
            "x": (r00, r10, self.yaw_x),
            "y": (r01, r11, self.yaw_y),
            "z": (r02, r12, self.yaw_z),
        }
        px, py, yaw_raw = axis_map[self.yaw_axis]
        proj = math.hypot(px, py)

        if proj < self.yaw_proj_min:
            if now - self.last_warn_t > 1.0:
                self.node.get_logger().warn(
                    "[YAW] proj too small axis={} proj={:.3f} < {:.3f}".format(
                        self.yaw_axis, proj, self.yaw_proj_min
                    )
                )
                self.last_warn_t = now
            self.dog_t = now
            return

        if self.dog_yaw is None:
            self.dog_yaw = yaw_raw
        else:
            d = norm_angle(yaw_raw - self.dog_yaw)
            self.dog_yaw = norm_angle(self.dog_yaw + self.yaw_alpha * d)

        self.dog_t = now

    def stale_items(self, now: float, max_age: float = 2.0) -> List[str]:
        items: List[str] = []
        if now - self.ball_t > max_age:
            items.append("ball({:.1f}s)".format(now - self.ball_t))
        if now - self.goal_t > max_age:
            items.append("goal({:.1f}s)".format(now - self.goal_t))
        if now - self.dog_t > max_age:
            items.append("dog({:.1f}s)".format(now - self.dog_t))
        return items

    def snapshot(
        self, goal_fallback_xy: Tuple[float, float]
    ) -> Optional[PerceptionSnapshot]:
        now = time.monotonic()
        if self.ball_xy is None or self.dog_xy is None or self.dog_yaw is None:
            return None

        goal_from_tracker = (
            self.goal_xy is not None and (now - self.goal_t) < 5.0
        )
        if goal_from_tracker and self.goal_xy is not None:
            gx, gy = self.goal_xy
        else:
            gx, gy = goal_fallback_xy

        return PerceptionSnapshot(
            t=now,
            dog_x=self.dog_xy[0],
            dog_y=self.dog_xy[1],
            dog_yaw=norm_angle(self.dog_yaw + self.yaw_offset),
            dog_yaw_x=self.yaw_x,
            dog_yaw_y=self.yaw_y,
            dog_yaw_z=self.yaw_z,
            ball_x=self.ball_xy[0],
            ball_y=self.ball_xy[1],
            ball_vx=self._ball_vx,
            ball_vy=self._ball_vy,
            goal_x=gx,
            goal_y=gy,
            goal_from_tracker=goal_from_tracker,
        )
