#!/usr/bin/env python3
"""Forward dash behavior for kick execution."""

import math

try:
    from .vrpn_perception import FAST_TROT, MotionCmd, PerceptionSnapshot, clamp, norm_angle
except ImportError:
    from vrpn_perception import FAST_TROT, MotionCmd, PerceptionSnapshot, clamp, norm_angle


class DashKicker:
    """Part 4: short forward dash while keeping heading near kick line."""

    def __init__(
        self,
        dash_speed: float,
        dash_duration: float,
        dash_yaw_gain: float,
        max_yaw_rate: float,
    ) -> None:
        self.dash_speed = dash_speed
        self.dash_duration = dash_duration
        self.dash_yaw_gain = dash_yaw_gain
        self.max_yaw_rate = max_yaw_rate

    def command(self, s: PerceptionSnapshot) -> MotionCmd:
        # Pure forward dash — lateral nudge is handled in BallKicker.
        return MotionCmd(FAST_TROT, self.dash_speed, 0.0, 0.0)
