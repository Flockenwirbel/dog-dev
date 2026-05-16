#!/usr/bin/env python3
"""Position adjustment before kick: align body with kick line."""

import math

try:
    from .vrpn_perception import MEDIUM_TROT, MotionCmd, PerceptionSnapshot, clamp, norm_angle
except ImportError:
    from vrpn_perception import MEDIUM_TROT, MotionCmd, PerceptionSnapshot, clamp, norm_angle


class AdjustResult:
    def __init__(
        self,
        cmd: MotionCmd,
        ready_dash: bool,
        line_err: float,
        ball_err: float,
        lateral_err: float,
        dist_ball: float,
    ) -> None:
        self.cmd = cmd
        self.ready_dash = ready_dash
        self.line_err = line_err
        self.ball_err = ball_err
        self.lateral_err = lateral_err
        self.dist_ball = dist_ball


class PositionAdjuster:
    """Part 3: adjust pose for kick line and kick window."""

    def __init__(
        self,
        yaw_gain: float,
        max_yaw_rate: float,
        kick_line_tol: float,
        kick_ball_tol: float,
        kick_lateral_tol: float,
        kick_trigger_dist: float,
        kick_min_dist: float,
        creep_speed: float = 0.35,
        lateral_gain: float = 0.9,
        max_lateral_speed: float = 0.20,
        ready_relax: float = 1.25,
        dist_relax: float = 0.08,
        consecutive_frames: int = 5,
    ) -> None:
        self.yaw_gain = yaw_gain
        self.max_yaw_rate = max_yaw_rate
        self.kick_line_tol = kick_line_tol
        self.kick_ball_tol = kick_ball_tol
        self.kick_lateral_tol = kick_lateral_tol
        self.kick_trigger_dist = kick_trigger_dist
        self.kick_min_dist = kick_min_dist
        self.creep_speed = creep_speed
        self.lateral_gain = lateral_gain
        self.max_lateral_speed = max_lateral_speed
        self.ready_relax = ready_relax
        self.dist_relax = dist_relax
        self.consecutive_frames = consecutive_frames
        self._aligned_count = 0

    def step(self, s: PerceptionSnapshot) -> AdjustResult:
        line_dx = s.goal_x - s.ball_x
        line_dy = s.goal_y - s.ball_y
        line_len = math.hypot(line_dx, line_dy)
        if line_len < 1e-6:
            line_ux, line_uy = 1.0, 0.0
        else:
            line_ux = line_dx / line_len
            line_uy = line_dy / line_len

        yaw_line = math.atan2(line_uy, line_ux)
        line_err = norm_angle(yaw_line - s.dog_yaw)

        yaw_ball = math.atan2(s.ball_y - s.dog_y, s.ball_x - s.dog_x)
        ball_err = norm_angle(yaw_ball - s.dog_yaw)

        dist_ball = math.hypot(s.dog_x - s.ball_x, s.dog_y - s.ball_y)

        # Signed cross-track distance from dog to the kick line (ball -> goal).
        # Positive means dog is on the line's left side.
        rel_x = s.dog_x - s.ball_x
        rel_y = s.dog_y - s.ball_y
        lateral_err = rel_x * (-line_uy) + rel_y * line_ux

        line_tol = self.kick_line_tol * self.ready_relax
        ball_tol = self.kick_ball_tol * self.ready_relax
        lateral_tol = self.kick_lateral_tol * self.ready_relax
        min_dist = max(0.0, self.kick_min_dist - self.dist_relax)
        max_dist = self.kick_trigger_dist + self.dist_relax

        aligned_now = (
            abs(line_err) < line_tol
            and abs(ball_err) < ball_tol
            and abs(lateral_err) < lateral_tol
            and min_dist < dist_ball < max_dist
        )

        if aligned_now:
            self._aligned_count += 1
        else:
            self._aligned_count = 0

        ready_dash = self._aligned_count >= self.consecutive_frames

        blend = 0.75 * line_err + 0.25 * ball_err
        vyaw = clamp(self.yaw_gain * blend, -self.max_yaw_rate, self.max_yaw_rate)

        # World-frame correction toward the kick line, then project into body frame.
        corr_wx = self.lateral_gain * (-lateral_err) * (-line_uy)
        corr_wy = self.lateral_gain * (-lateral_err) * line_ux

        cyaw = math.cos(s.dog_yaw)
        syaw = math.sin(s.dog_yaw)
        vy_body = corr_wx * syaw - corr_wy * cyaw
        # On this platform's motion interface, lateral command sign is opposite
        # to this geometric body projection, so invert it for correction.
        vy = clamp(-vy_body, -self.max_lateral_speed, self.max_lateral_speed)

        forward_ok = abs(lateral_err) < max(0.18, 2.0 * self.kick_lateral_tol)
        if dist_ball > self.kick_trigger_dist and forward_ok:
            # Too far — creep forward toward ball.
            vx = self.creep_speed
        elif dist_ball < self.kick_trigger_dist * 0.75 and forward_ok:
            # Too close — back up to get runway for dash acceleration.
            vx = -self.creep_speed * 0.5
        else:
            vx = 0.0
        return AdjustResult(
            cmd=MotionCmd(MEDIUM_TROT, vx, vy, vyaw),
            ready_dash=ready_dash,
            line_err=line_err,
            ball_err=ball_err,
            lateral_err=lateral_err,
            dist_ball=dist_ball,
        )
