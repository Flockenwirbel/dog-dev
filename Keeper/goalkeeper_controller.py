#!/usr/bin/env python3
"""Goalkeeper controller with state machine: GUARD, INTERCEPT, PUSH.

Controls a robot dog to defend a goal by:
  - staying within 1 m in front of the goal when the ball is far away (GUARD),
  - moving to the ball's predicted interception point when the ball comes
    within 2 m of the goal (INTERCEPT),
  - pushing the ball away at 120 deg from the ball's motion direction once
    the dog makes contact with the ball (PUSH).

The dog's head (yaw) always faces the ball in all states.
"""

import math
import time
from typing import Optional, Tuple

from demo_python_pkg.vrpn_perception import MotionCmd, clamp, norm_angle


class GoalkeeperController:
    GUARD = 0
    INTERCEPT = 1
    PUSH = 2

    def __init__(
        self,
        guard_dist: float = 0.7,
        max_guard_dist: float = 1.0,
        intercept_trigger: float = 2.0,
        contact_dist: float = 0.4,
        push_speed: float = 1.5,
        push_angle_deg: float = 120.0,
        push_duration: float = 0.5,
        track_speed: float = 0.8,
        lateral_speed: float = 0.5,
        yaw_gain: float = 0.6,
        max_yaw_rate: float = 1.0,
        gait_guard: int = 303, 
        gait_intercept: int = 305,
        gait_push: int = 305,
    ) -> None:
        self.guard_dist = guard_dist
        self.max_guard_dist = max_guard_dist
        self.intercept_trigger = intercept_trigger
        self.contact_dist = contact_dist
        self.push_speed = push_speed
        self.push_angle_rad = math.radians(push_angle_deg)
        self.push_duration = push_duration
        self.track_speed = track_speed
        self.lateral_speed = lateral_speed
        self.yaw_gain = yaw_gain
        self.max_yaw_rate = max_yaw_rate
        self.gait_guard = gait_guard
        self.gait_intercept = gait_intercept
        self.gait_push = gait_push

        self._state = self.GUARD
        self._push_start_t: float = 0.0
        self._state_name = {0: "GUARD", 1: "INTERCEPT", 2: "PUSH"}

    @property
    def state(self) -> int:
        return self._state

    @property
    def state_name(self) -> str:
        return self._state_name.get(self._state, "UNKNOWN")

    def step(
        self,
        dog_pos: Tuple[float, float, float],
        ball_pos: Tuple[float, float],
        ball_vel: Tuple[float, float],
        goal_pos: Tuple[float, float],
        t: float,
    ) -> MotionCmd:
        dog_x, dog_y, dog_yaw = dog_pos
        ball_x, ball_y = ball_pos
        ball_vx, ball_vy = ball_vel
        goal_x, goal_y = goal_pos

        ball_to_goal = math.hypot(ball_x - goal_x, ball_y - goal_y)
        dog_to_ball = math.hypot(dog_x - ball_x, dog_y - ball_y)
        ball_speed = math.hypot(ball_vx, ball_vy)

        # ---------- state transitions ----------
        if self._state == self.PUSH:
            if t - self._push_start_t > self.push_duration:
                self._state = self.INTERCEPT
        elif dog_to_ball < self.contact_dist:
            self._state = self.PUSH
            self._push_start_t = t
        elif ball_to_goal < self.intercept_trigger:
            self._state = self.INTERCEPT
        else:
            self._state = self.GUARD


        # ---------- compute motion ----------
        if self._state == self.GUARD:
            return self._guard_step(
                dog_x, dog_y, dog_yaw, ball_x, ball_y, goal_x, goal_y
            )
        elif self._state == self.INTERCEPT:
            return self._intercept_step(
                dog_x,
                dog_y,
                dog_yaw,
                ball_x,
                ball_y,
                ball_vx,
                ball_vy,
                ball_speed,
                goal_x,
                goal_y,
            )
        else:  # PUSH
            return self._push_step(
                dog_x,
                dog_y,
                dog_yaw,
                ball_x,
                ball_y,
                ball_vx,
                ball_vy,
                ball_speed,
            )

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    #  GUARD - move directly to position in front of goal
    # ------------------------------------------------------------------
    def _guard_step(
        self,
        dog_x: float,
        dog_y: float,
        dog_yaw: float,
        ball_x: float,
        ball_y: float,
        goal_x: float,
        goal_y: float,
    ) -> MotionCmd:
        # Target: directly in front of goal
        if ball_y < goal_y:
            target_y = goal_y - self.guard_dist
        else:
            target_y = goal_y + self.guard_dist
        target_x = goal_x

        move_x = target_x - dog_x
        move_y = target_y - dog_y
        dist = math.hypot(move_x, move_y)

        # Arrived: stand still and face the ball
        if dist < 0.2:
            ball_yaw = math.atan2(ball_y - dog_y, ball_x - dog_x)
            yaw_err = norm_angle(ball_yaw - dog_yaw)
            vyaw = clamp(self.yaw_gain * yaw_err, -self.max_yaw_rate, self.max_yaw_rate)
            return MotionCmd(308, 0.0, 0.0, vyaw)

        # World-frame velocity toward target
        speed = self.track_speed
        if dist < 0.5:
            speed = 0.3
        wx = move_x / dist * speed
        wy = move_y / dist * speed

        # Convert to body-frame so dog walks forward
        cos_yaw = math.cos(dog_yaw)
        sin_yaw = math.sin(dog_yaw)
        vx = wx * cos_yaw + wy * sin_yaw
        vy = -wx * sin_yaw + wy * cos_yaw

        return MotionCmd(308, vx, vy, 0.0)


    # ------------------------------------------------------------------
    #  INTERCEPT - predict ball trajectory and move to interception point
    # ------------------------------------------------------------------
    def _intercept_step(
        self,
        dog_x: float,
        dog_y: float,
        dog_yaw: float,
        ball_x: float,
        ball_y: float,
        ball_vx: float,
        ball_vy: float,
        ball_speed: float,
        goal_x: float,
        goal_y: float,
    ) -> MotionCmd:
        dx_goal = ball_x - goal_x
        dy_goal = ball_y - goal_y
        d_goal = math.hypot(dx_goal, dy_goal)

        if d_goal < 1e-6:
            return MotionCmd(self.gait_intercept, 0.0, 0.0, 0.0)

        ux = dx_goal / d_goal
        uy = dy_goal / d_goal

        if ball_speed < 0.2:
            target_x = goal_x + ux * self.guard_dist
            target_y = goal_y + uy * self.guard_dist
            return self._move_toward(
                dog_x, dog_y, dog_yaw, target_x, target_y, ball_x, ball_y,
                speed=self.track_speed, gait=self.gait_intercept
            )

        nx = ux
        ny = uy
        px = goal_x + ux * self.guard_dist
        py = goal_y + uy * self.guard_dist

        denom = nx * ball_vx + ny * ball_vy
        if abs(denom) < 1e-6:
            target_x = ball_x
            target_y = ball_y
        else:
            numer = nx * (px - ball_x) + ny * (py - ball_y)
            t_cross = numer / denom
            t_pred = max(0.0, min(t_cross, 1.5))
            target_x = ball_x + t_pred * ball_vx
            target_y = ball_y + t_pred * ball_vy

        target_dx = target_x - goal_x
        target_dy = target_y - goal_y
        target_dist = math.hypot(target_dx, target_dy)

        if target_dist < 1e-6:
            target_dist = 1e-6
            target_dx = ux * 1e-6
            target_dy = uy * 1e-6

        if target_dist > self.max_guard_dist:
            target_x = goal_x + target_dx / target_dist * self.max_guard_dist
            target_y = goal_y + target_dy / target_dist * self.max_guard_dist
        elif target_dist < self.guard_dist * 0.5:
            target_x = goal_x + target_dx / target_dist * self.guard_dist * 0.5
            target_y = goal_y + target_dy / target_dist * self.guard_dist * 0.5

        speed = min(self.lateral_speed * 1.4, self.track_speed)
        if ball_speed > 0.2:
            bias_x = ball_vx / ball_speed * 0.15
            bias_y = ball_vy / ball_speed * 0.15
            target_x += bias_x
            target_y += bias_y

        return self._move_toward(
            dog_x, dog_y, dog_yaw, target_x, target_y, ball_x, ball_y,
            speed=speed, gait=self.gait_intercept
        )

    # ------------------------------------------------------------------
    #  PUSH - dash toward the ball to kick it away from goal
    # ------------------------------------------------------------------
    def _push_step(
        self,
        dog_x: float,
        dog_y: float,
        dog_yaw: float,
        ball_x: float,
        ball_y: float,
        ball_vx: float,
        ball_vy: float,
        ball_speed: float,
    ) -> MotionCmd:
        # Dash direction: from dog toward ball
        dx = ball_x - dog_x
        dy = ball_y - dog_y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return MotionCmd(self.gait_push, 0.0, 0.0, 0.0)

        wx = dx / dist * self.push_speed
        wy = dy / dist * self.push_speed

        cos_yaw = math.cos(dog_yaw)
        sin_yaw = math.sin(dog_yaw)
        vx = wx * cos_yaw + wy * sin_yaw
        vy = -wx * sin_yaw + wy * cos_yaw

        # Face the ball while dashing
        desired_yaw = math.atan2(dy, dx)
        yaw_error = norm_angle(desired_yaw - dog_yaw)
        vyaw = clamp(self.yaw_gain * yaw_error, -self.max_yaw_rate, self.max_yaw_rate)

        return MotionCmd(self.gait_push, vx, vy, vyaw)

    # ------------------------------------------------------------------
    #  Helper - compute a MotionCmd that moves the dog toward a target
    # ------------------------------------------------------------------
    def _move_toward(
        self,
        dog_x: float,
        dog_y: float,
        dog_yaw: float,
        target_x: float,
        target_y: float,
        face_x: float,
        face_y: float,
        speed: float,
        gait: int,
    ) -> MotionCmd:
        move_x = target_x - dog_x
        move_y = target_y - dog_y
        move_dist = math.hypot(move_x, move_y)

        # Decelerate near target
        if move_dist < 0.3:
            speed *= 0.25
        elif move_dist < 0.6:
            speed *= 0.5
        elif move_dist < 1.0:
            speed *= 0.75

        if move_dist > 1e-3:
            wx = move_x / move_dist * speed
            wy = move_y / move_dist * speed
        else:
            wx, wy = 0.0, 0.0

        cos_yaw = math.cos(dog_yaw)
        sin_yaw = math.sin(dog_yaw)
        vx = wx * cos_yaw + wy * sin_yaw
        vy = -wx * sin_yaw + wy * cos_yaw

        desired_yaw = math.atan2(face_y - dog_y, face_x - dog_x)
        yaw_error = norm_angle(desired_yaw - dog_yaw)
        vyaw = clamp(self.yaw_gain * yaw_error, -self.max_yaw_rate, self.max_yaw_rate)

        return MotionCmd(gait, vx, vy, vyaw)
