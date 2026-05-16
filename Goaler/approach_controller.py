#!/usr/bin/env python3
"""Approach behavior: navigate to prekick point while facing ball."""

import math
from typing import Optional, Tuple

try:
    from .vrpn_perception import (
        FAST_TROT,
        MEDIUM_TROT,
        MotionCmd,
        PerceptionSnapshot,
        clamp,
        norm_angle,
    )
except ImportError:
    from vrpn_perception import (
        FAST_TROT,
        MEDIUM_TROT,
        MotionCmd,
        PerceptionSnapshot,
        clamp,
        norm_angle,
    )


class ApproachResult:
    def __init__(
        self,
        cmd: MotionCmd,
        ready_align: bool,
        dist_ball: float,
        dist_prekick: float,
        yaw_err_ball: float,
        prekick_ang_err: float,
    ) -> None:
        self.cmd = cmd
        self.ready_align = ready_align
        self.dist_ball = dist_ball
        self.dist_prekick = dist_prekick
        self.yaw_err_ball = yaw_err_ball
        self.prekick_ang_err = prekick_ang_err


class ApproachController:
    """Navigate dog to the prekick point behind ball while yaw-facing ball.

    No orbit mode — just a single unified controller:
    1. World-frame velocity toward prekick point
    2. Yaw toward ball
    3. Repulsive force from ball if too close
    4. Transform to body frame
    """

    def __init__(
        self,
        approach_speed: float,
        side_step_speed: float,
        yaw_gain: float,
        max_yaw_rate: float,
        prekick_offset: float,
        prekick_dist_tol: float,
        wrap_dist: float = 0.70,
        behind_margin_y: float = 0.08,
        line_align_tol: float = 0.35,
        orbit_radius_extra: float = 0.30,
        yaw_recover_thresh: float = 0.35,
        yaw_recover_speed: float = 0.10,
        lateral_deadband: float = 0.08,
    ) -> None:
        self.approach_speed = approach_speed
        self.side_step_speed = side_step_speed
        self.yaw_gain = yaw_gain
        self.max_yaw_rate = max_yaw_rate
        self.prekick_offset = prekick_offset
        self.prekick_dist_tol = prekick_dist_tol

        # Fast approach from far away.
        self.fast_approach_speed = 3.00
        self.fast_approach_threshold = 1.50

        # Ball avoidance: repulsion + standoff.
        self.min_ball_dist = 0.60
        self.repulsion_gain = 1.00
        self.standoff_trigger_dist = 1.20    # max ball-dog distance for standoff
        self.standoff_lateral = 0.80          # lateral offset from prekick
        self.standoff_arrive_tol = 0.35       # arrive tolerance
        self.ready_angle_tol = math.radians(28.0)
        self._in_standoff = False
        self._standoff_xyz = (0.0, 0.0)

    def reset(self) -> None:
        """Call after cooldown."""
        self._in_standoff = False
        self._standoff_xyz = (0.0, 0.0)

    def _prekick_point(self, s: PerceptionSnapshot) -> Optional[Tuple[float, float]]:
        dx = s.ball_x - s.goal_x
        dy = s.ball_y - s.goal_y
        d = math.hypot(dx, dy)
        if d < 1e-4:
            return None
        return (
            s.ball_x + dx / d * self.prekick_offset,
            s.ball_y + dy / d * self.prekick_offset,
        )

    @staticmethod
    def _toward(sx: float, sy: float, tx: float, ty: float, speed: float) -> Tuple[float, float]:
        """World-frame unit velocity toward (tx, ty) scaled by speed."""
        dx = tx - sx
        dy = ty - sy
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return (0.0, 0.0)
        return (dx / d * speed, dy / d * speed)

    @staticmethod
    def _to_body(wx: float, wy: float, cyaw: float, syaw: float) -> Tuple[float, float]:
        """World→body frame: (forward, left)."""
        return (wx * cyaw + wy * syaw, -wx * syaw + wy * cyaw)

    def step(self, s: PerceptionSnapshot) -> ApproachResult:
        tx_ty = self._prekick_point(s)
        if tx_ty is None:
            return ApproachResult(
                cmd=MotionCmd(FAST_TROT, 0.0, 0.0, 0.0),
                ready_align=False,
                dist_ball=999.0,
                dist_prekick=999.0,
                yaw_err_ball=0.0,
                prekick_ang_err=0.0,
            )

        tx, ty = tx_ty
        dist_ball = math.hypot(s.dog_x - s.ball_x, s.dog_y - s.ball_y)
        dist_prekick = math.hypot(s.dog_x - tx, s.dog_y - ty)

        yaw_to_ball = math.atan2(s.ball_y - s.dog_y, s.ball_x - s.dog_x)
        yaw_err_ball = norm_angle(yaw_to_ball - s.dog_yaw)

        # prekick_ang_err: angular difference between dog→ball and ball→prekick
        bd_x = s.dog_x - s.ball_x
        bd_y = s.dog_y - s.ball_y
        bt_x = tx - s.ball_x
        bt_y = ty - s.ball_y
        if math.hypot(bd_x, bd_y) > 1e-6 and math.hypot(bt_x, bt_y) > 1e-6:
            theta_dog = math.atan2(bd_y, bd_x)
            theta_target = math.atan2(bt_y, bt_x)
            prekick_ang_err = norm_angle(theta_target - theta_dog)
        else:
            prekick_ang_err = 0.0

        cyaw = math.cos(s.dog_yaw)
        syaw = math.sin(s.dog_yaw)

        # --- Ready check: near prekick point and facing ball ---
        prekick_near = max(self.prekick_dist_tol, 0.35)
        if dist_prekick < prekick_near and abs(prekick_ang_err) < self.ready_angle_tol:
            vyaw = clamp(self.yaw_gain * yaw_err_ball, -self.max_yaw_rate, self.max_yaw_rate)
            return ApproachResult(
                cmd=MotionCmd(MEDIUM_TROT, 0.0, 0.0, vyaw),
                ready_align=True,
                dist_ball=dist_ball,
                dist_prekick=dist_prekick,
                yaw_err_ball=yaw_err_ball,
                prekick_ang_err=prekick_ang_err,
            )

        # --- Standoff: if ball is between dog and prekick, go lateral first ---
        # dot(dog-ball, prekick-ball) < 0 → dog and prekick on opposite sides of ball
        ball_in_way = (bd_x * bt_x + bd_y * bt_y < 0) and dist_ball < self.standoff_trigger_dist

        if ball_in_way and not self._in_standoff:
            # Enter standoff: pick a lateral point near prekick
            kdx = s.goal_x - s.ball_x
            kdy = s.goal_y - s.ball_y
            kd = math.hypot(kdx, kdy)
            if kd > 1e-6:
                perp_x, perp_y = -kdy / kd, kdx / kd  # 90deg left of kick line
            else:
                perp_x, perp_y = 1.0, 0.0
            # Pick side closer to dog
            dot_perp = (s.dog_x - tx) * perp_x + (s.dog_y - ty) * perp_y
            sign = 1.0 if dot_perp >= 0.0 else -1.0
            self._standoff_xyz = (
                tx + perp_x * sign * self.standoff_lateral,
                ty + perp_y * sign * self.standoff_lateral,
            )
            self._in_standoff = True

        if self._in_standoff:
            sx, sy = self._standoff_xyz
            dist_standoff = math.hypot(s.dog_x - sx, s.dog_y - sy)
            if dist_standoff < self.standoff_arrive_tol or not ball_in_way:
                self._in_standoff = False
            else:
                tx, ty = sx, sy  # redirect target to standoff
                dist_prekick = dist_standoff  # log as distance to current target

        # --- Unified approach: go toward target, face ball ---
        # World-frame velocity toward target (prekick or standoff).
        speed = self.fast_approach_speed if dist_ball > self.fast_approach_threshold else self.approach_speed
        tow_x, tow_y = self._toward(s.dog_x, s.dog_y, tx, ty, speed)

        # Repulsive force from ball when too close.
        if dist_ball < self.min_ball_dist and dist_ball > 1e-6:
            penetration = self.min_ball_dist - dist_ball
            repel = penetration * self.repulsion_gain
            away_x = (s.dog_x - s.ball_x) / dist_ball * repel
            away_y = (s.dog_y - s.ball_y) / dist_ball * repel
            wx = tow_x + away_x
            wy = tow_y + away_y
        else:
            wx, wy = tow_x, tow_y

        # Transform to body frame.
        vx_body, vy_body = self._to_body(wx, wy, cyaw, syaw)

        # Scale forward speed by how well we face the ball.
        face_factor = max(0.0, math.cos(yaw_err_ball))
        vx_body *= 0.35 + 0.65 * face_factor

        vyaw = clamp(self.yaw_gain * yaw_err_ball, -self.max_yaw_rate, self.max_yaw_rate)

        return ApproachResult(
            cmd=MotionCmd(FAST_TROT, vx_body, vy_body, vyaw),
            ready_align=False,
            dist_ball=dist_ball,
            dist_prekick=dist_prekick,
            yaw_err_ball=yaw_err_ball,
            prekick_ang_err=prekick_ang_err,
        )
