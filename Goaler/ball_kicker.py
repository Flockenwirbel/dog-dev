#!/usr/bin/env python3
"""Ball kicker main node: kick logic + state machine wiring."""

import json
import math
import time
from typing import Dict

import rclpy
from protocol.msg import MotionServoCmd
from rclpy.node import Node
from std_msgs.msg import String

try:
    from .approach_controller import ApproachController
    from .dash_kicker import DashKicker
    from .position_adjuster import PositionAdjuster
    from .vrpn_perception import (
        CMD_DATA,
        GAIT_STANDARD,
        MEDIUM_TROT,
        MotionCmd,
        PerceptionSnapshot,
        VrpnPerception,
        clamp,
    )
except ImportError:
    from approach_controller import ApproachController
    from dash_kicker import DashKicker
    from position_adjuster import PositionAdjuster
    from vrpn_perception import (
        CMD_DATA,
        GAIT_STANDARD,
        MEDIUM_TROT,
        MotionCmd,
        PerceptionSnapshot,
        VrpnPerception,
        clamp,
    )


class BallKicker(Node):
    APPROACH = "approach"
    ALIGN_KICK = "align_kick"
    DASH = "dash"
    COOLDOWN = "cooldown"

    def __init__(self) -> None:
        super().__init__("ball_kicker")

        # Trackers
        self.declare_parameter("dog_name", "XiaoChuan_Sun")
        self.declare_parameter("ball_tracker", "ball")
        self.declare_parameter("dog_tracker", "XiaoChuan_Sun")
        self.declare_parameter("goal_tracker", "goal_right")

        # Perception parameters
        self.declare_parameter("forward_yaw_axis", "x")
        self.declare_parameter("yaw_proj_min", 0.10)
        self.declare_parameter("yaw_filter_alpha", 0.35)
        self.declare_parameter("stop_motion_id", MEDIUM_TROT)

        # Approach parameters
        self.declare_parameter("prekick_offset", 0.75)
        self.declare_parameter("goto_dist_tol", 0.25)
        self.declare_parameter("approach_speed", 1.00)
        self.declare_parameter("side_step_speed", 0.45)
        self.declare_parameter("wrap_dist", 0.70)
        self.declare_parameter("approach_behind_margin_y", 0.08)
        self.declare_parameter("approach_line_align_tol", 0.35)
        self.declare_parameter("approach_orbit_radius_extra", 0.30)
        self.declare_parameter("approach_yaw_recover_thresh", 0.35)
        self.declare_parameter("approach_yaw_recover_speed", 0.18)
        self.declare_parameter("approach_lateral_deadband", 0.08)
        self.declare_parameter("yaw_gain", 3.0)
        self.declare_parameter("max_yaw_rate", 0.6)

        # Kick parameters
        self.declare_parameter("kick_line_tol", 0.0873)
        self.declare_parameter("kick_ball_tol", 0.0873)
        self.declare_parameter("kick_lateral_tol", 0.08)
        self.declare_parameter("kick_trigger_dist", 0.90)
        self.declare_parameter("kick_min_dist", 0.30)
        self.declare_parameter("align_lateral_gain", 1.5)
        self.declare_parameter("align_max_lateral_speed", 0.25)
        self.declare_parameter("align_ready_relax", 1.00)
        self.declare_parameter("align_dist_relax", 0.12)
        self.declare_parameter("dash_centroid_z", 0.10)
        self.declare_parameter("dash_nudge_vy", 0.15)
        self.declare_parameter("dash_nudge_duration", 0.30)
        self.declare_parameter("align_consecutive_frames", 3)
        self.declare_parameter("dash_speed", 2.00)
        self.declare_parameter("dash_duration", 1.00)
        self.declare_parameter("dash_yaw_gain", 1.5)

        # Safety / timing
        self.declare_parameter("safe_ball_dist", 0.35)
        self.declare_parameter("dash_stop_goal_dist", 0.50)
        self.declare_parameter("align_timeout", 6.0)
        self.declare_parameter("approach_timeout", 30.0)
        self.declare_parameter("cooldown_duration", 2.0)

        p = self.get_parameter
        self.dog_name = p("dog_name").value
        ball_tracker = p("ball_tracker").value
        dog_tracker = p("dog_tracker").value
        goal_tracker = p("goal_tracker").value

        self.safe_ball_dist = float(p("safe_ball_dist").value)
        self.dash_stop_goal_dist = float(p("dash_stop_goal_dist").value)
        self.align_timeout = float(p("align_timeout").value)
        self.approach_timeout = float(p("approach_timeout").value)
        self.cooldown_duration = float(p("cooldown_duration").value)
        self.stop_motion_id = int(p("stop_motion_id").value)

        yaw_axis = str(p("forward_yaw_axis").value).lower()
        if yaw_axis not in ("x", "y", "z"):
            yaw_axis = "y"

        self.perception = VrpnPerception(
            node=self,
            ball_tracker=ball_tracker,
            dog_tracker=dog_tracker,
            goal_tracker=goal_tracker,
            yaw_axis=yaw_axis,
            yaw_proj_min=float(p("yaw_proj_min").value),
            yaw_alpha=clamp(float(p("yaw_filter_alpha").value), 0.0, 1.0),
        )

        self.approach = ApproachController(
            approach_speed=float(p("approach_speed").value),
            side_step_speed=float(p("side_step_speed").value),
            yaw_gain=float(p("yaw_gain").value),
            max_yaw_rate=float(p("max_yaw_rate").value),
            prekick_offset=float(p("prekick_offset").value),
            prekick_dist_tol=float(p("goto_dist_tol").value),
            wrap_dist=float(p("wrap_dist").value),
            behind_margin_y=float(p("approach_behind_margin_y").value),
            line_align_tol=float(p("approach_line_align_tol").value),
            orbit_radius_extra=float(p("approach_orbit_radius_extra").value),
            yaw_recover_thresh=float(p("approach_yaw_recover_thresh").value),
            yaw_recover_speed=float(p("approach_yaw_recover_speed").value),
            lateral_deadband=float(p("approach_lateral_deadband").value),
        )

        self.position_adjuster = PositionAdjuster(
            yaw_gain=float(p("yaw_gain").value),
            max_yaw_rate=float(p("max_yaw_rate").value),
            kick_line_tol=float(p("kick_line_tol").value),
            kick_ball_tol=float(p("kick_ball_tol").value),
            kick_lateral_tol=float(p("kick_lateral_tol").value),
            kick_trigger_dist=float(p("kick_trigger_dist").value),
            kick_min_dist=float(p("kick_min_dist").value),
            lateral_gain=float(p("align_lateral_gain").value),
            max_lateral_speed=float(p("align_max_lateral_speed").value),
            ready_relax=float(p("align_ready_relax").value),
            dist_relax=float(p("align_dist_relax").value),
            consecutive_frames=int(p("align_consecutive_frames").value),
        )

        self.dash_kicker = DashKicker(
            dash_speed=float(p("dash_speed").value),
            dash_duration=float(p("dash_duration").value),
            dash_yaw_gain=float(p("dash_yaw_gain").value),
            max_yaw_rate=float(p("max_yaw_rate").value),
        )

        self.goal_fixed_xy = (0.0, 4.2)

        self.pub_cmd = self.create_publisher(
            MotionServoCmd,
            "/{}/motion_servo_cmd".format(self.dog_name),
            10,
        )
        self.pub_state = self.create_publisher(
            String,
            "/{}/ball_kicker_state".format(self.dog_name),
            10,
        )

        self.state = self.APPROACH
        self.state_t0 = time.monotonic()
        self.last_cmd = MotionCmd(MEDIUM_TROT, 0.0, 0.0, 0.0)
        self._stop_sent = False
        self._last_log_t = 0.0

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            "BallKicker v8(split-5) | axis={} approach={:.2f} dash={:.2f}".format(
                yaw_axis,
                float(p("approach_speed").value),
                float(p("dash_speed").value),
            )
        )

    def _cmd(self, cmd: MotionCmd, gait_value: int = GAIT_STANDARD, centroid_z: float = 0.0) -> None:
        msg = MotionServoCmd()
        msg.motion_id = int(cmd.gait)
        msg.cmd_type = CMD_DATA
        msg.value = gait_value
        msg.vel_des = [float(cmd.vx), float(cmd.vy), float(cmd.vyaw)]
        msg.step_height = [0.05, 0.05]
        if centroid_z > 0.0:
            msg.pos_des = [0.0, 0.0, centroid_z]
        self.pub_cmd.publish(msg)
        self.last_cmd = cmd
        self._stop_sent = False

    def _stand_once(self) -> None:
        if self._stop_sent:
            return
        msg = MotionServoCmd()
        msg.motion_id = self.stop_motion_id
        msg.cmd_type = CMD_DATA
        msg.value = GAIT_STANDARD
        msg.vel_des = [0.0, 0.0, 0.0]
        msg.step_height = [0.05, 0.05]
        self.pub_cmd.publish(msg)
        self._stop_sent = True

    def _step_in_place(self) -> None:
        self._cmd(MotionCmd(MEDIUM_TROT, 0.0, 0.0, 0.0))

    def _set_state(self, new_state: str, reason: str) -> None:
        if new_state != self.state:
            self.get_logger().info("state: {} -> {} ({})".format(self.state, new_state, reason))
        self.state = new_state
        self.state_t0 = time.monotonic()

    def _log(self, text: str, period_s: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._last_log_t > period_s:
            self.get_logger().info(text)
            self._last_log_t = now

    def _publish_state(self, s: PerceptionSnapshot, extra: Dict[str, float]) -> None:
        payload = {
            "state": self.state,
            "dog": {"x": s.dog_x, "y": s.dog_y, "yaw": s.dog_yaw},
            "ball": {"x": s.ball_x, "y": s.ball_y},
            "goal": {"x": s.goal_x, "y": s.goal_y, "from_tracker": s.goal_from_tracker},
            "yaw_candidates": {
                "x": s.dog_yaw_x,
                "y": s.dog_yaw_y,
                "z": s.dog_yaw_z,
            },
            "cmd": {
                "gait": self.last_cmd.gait,
                "vx": self.last_cmd.vx,
                "vy": self.last_cmd.vy,
                "vyaw": self.last_cmd.vyaw,
            },
            "extra": extra,
            "t": s.t,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_state.publish(msg)

    def _prekick_point(self, s: PerceptionSnapshot):
        dx = s.ball_x - s.goal_x
        dy = s.ball_y - s.goal_y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return None
        return (
            s.ball_x + dx / d * self.approach.prekick_offset,
            s.ball_y + dy / d * self.approach.prekick_offset,
        )

    def _tick(self) -> None:
        now = time.monotonic()

        snapshot = self.perception.snapshot(self.goal_fixed_xy)
        if snapshot is None:
            self._step_in_place()
            self._log("Waiting VRPN snapshot, keep stepping in place", period_s=2.0)
            return

        stale = self.perception.stale_items(now, max_age=2.0)
        stale = [item for item in stale if not item.startswith("goal(")]
        if stale:
            self._log("Stale VRPN: {}".format(" ".join(stale)), period_s=3.0)
            self._step_in_place()
            return

        dist_ball = math.hypot(snapshot.dog_x - snapshot.ball_x, snapshot.dog_y - snapshot.ball_y)

        if self.state == self.APPROACH:
            if now - self.state_t0 > self.approach_timeout:
                self.get_logger().warn(
                    "Approach timeout ({:.0f}s), re-approach".format(self.approach_timeout)
                )
                self._set_state(self.APPROACH, "approach_timeout")
                self._publish_state(snapshot, {"dist_ball": dist_ball, "timeout": 1.0})
                return

            res = self.approach.step(snapshot)
            self._cmd(res.cmd)

            prekick = self._prekick_point(snapshot)
            if prekick is None:
                pre_x = 0.0
                pre_y = 0.0
            else:
                pre_x, pre_y = prekick

            self._log(
                "[approach] ball_d={:.2f} prekick_d={:.2f} prekick_ang={:+.1f}deg yaw_err_ball={:.1f}deg vx={:.2f} vy={:.2f} yaw={:+.2f} | dog=({:+.2f},{:+.2f}) ball=({:+.2f},{:+.2f}) prekick=({:+.2f},{:+.2f})".format(
                    res.dist_ball,
                    res.dist_prekick,
                    math.degrees(res.prekick_ang_err),
                    math.degrees(res.yaw_err_ball),
                    res.cmd.vx,
                    res.cmd.vy,
                    res.cmd.vyaw,
                    snapshot.dog_x,
                    snapshot.dog_y,
                    snapshot.ball_x,
                    snapshot.ball_y,
                    pre_x,
                    pre_y,
                )
            )

            if res.ready_align:
                self._set_state(self.ALIGN_KICK, "reached prekick and facing ball")

            self._publish_state(
                snapshot,
                {
                    "dist_ball": res.dist_ball,
                    "dist_prekick": res.dist_prekick,
                    "prekick_ang_err_deg": math.degrees(res.prekick_ang_err),
                    "yaw_err_ball_deg": math.degrees(res.yaw_err_ball),
                },
            )
            return

        if self.state == self.ALIGN_KICK:
            if dist_ball < self.safe_ball_dist * 0.75:
                self._set_state(self.APPROACH, "too close to ball, re-approach")
                self._publish_state(snapshot, {"dist_ball": dist_ball})
                return

            if now - self.state_t0 > self.align_timeout:
                self._set_state(self.APPROACH, "align timeout")
                self._publish_state(snapshot, {"dist_ball": dist_ball})
                return

            align = self.position_adjuster.step(snapshot)
            if align.ready_dash:
                self._set_state(self.DASH, "kick aligned")
                dash_cmd = self.dash_kicker.command(snapshot)
                self._cmd(dash_cmd, gait_value=0)
            else:
                self._cmd(align.cmd)

            self._log(
                "[align] line={:.1f}deg ball={:.1f}deg lat={:+.2f}m ball_d={:.2f} ready={} vx={:.2f} vy={:+.2f} yaw={:+.2f}".format(
                    math.degrees(align.line_err),
                    math.degrees(align.ball_err),
                    align.lateral_err,
                    align.dist_ball,
                    align.ready_dash,
                    self.last_cmd.vx,
                    self.last_cmd.vy,
                    self.last_cmd.vyaw,
                )
            )

            self._publish_state(
                snapshot,
                {
                    "line_err_deg": math.degrees(align.line_err),
                    "ball_err_deg": math.degrees(align.ball_err),
                    "lateral_err_m": align.lateral_err,
                    "dist_ball": align.dist_ball,
                    "ready_dash": 1.0 if align.ready_dash else 0.0,
                },
            )
            return

        if self.state == self.DASH:
            dist_goal = math.hypot(snapshot.dog_x - snapshot.goal_x, snapshot.dog_y - snapshot.goal_y)
            if dist_goal <= self.dash_stop_goal_dist:
                self._stand_once()
                self._log("[dash] reached goal ({:.2f}m), standing".format(dist_goal))
                self._publish_state(snapshot, {"dist_goal": dist_goal, "done": 1.0})
                return
            if dist_ball > 1.50:
                dt = now - self.state_t0
                if dt >= self.dash_kicker.dash_duration:
                    self._set_state(self.COOLDOWN, "kick_done")
                    self._publish_state(snapshot, {"dist_ball": dist_ball, "kick_done": 1.0})
                    return
                else:
                    self.get_logger().warn(
                        "Ball lost during dash ({:.2f}m), re-approach".format(dist_ball)
                    )
                    self.approach.reset()
                    self._set_state(self.APPROACH, "ball_lost_during_dash")
                    self._publish_state(snapshot, {"dist_ball": dist_ball})
                    return

            dz = float(self.get_parameter("dash_centroid_z").value)
            dash_cmd = self.dash_kicker.command(snapshot)
            dt = now - self.state_t0
            if dt < float(self.get_parameter("dash_nudge_duration").value):
                nudge = float(self.get_parameter("dash_nudge_vy").value)
                dash_cmd = MotionCmd(dash_cmd.gait, dash_cmd.vx, nudge, dash_cmd.vyaw)
            self._cmd(dash_cmd, gait_value=0, centroid_z=dz)

            self._log(
                "[dash] vx={:.1f} goal_d={:.2f} ball_d={:.2f}".format(
                    self.dash_kicker.dash_speed,
                    dist_goal,
                    dist_ball,
                )
            )

            self._publish_state(snapshot, {"dist_goal": dist_goal, "dist_ball": dist_ball})
            return

        if self.state == self.COOLDOWN:
            if now - self.state_t0 > self.cooldown_duration:
                self.approach.reset()
                self._set_state(self.APPROACH, "cooldown_done")
            else:
                self._stand_once()
            return

        # Fallback: stand
        self._stand_once()
        return

    def destroy_node(self) -> bool:
        self._stand_once()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BallKicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
