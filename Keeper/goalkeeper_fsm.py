#!/usr/bin/env python3
"""Goalkeeper FSM: GO_AROUND → APPROACH → READY → KICK.

Follows demo_python_pkg pattern: VrpnPerception + MotionCmd + FSM orchestrator.
"""

import math
import time

import rclpy
from protocol.msg import MotionServoCmd
from protocol.srv import MotionResultCmd
from rclpy.node import Node

from .vrpn_perception import (
    CMD_DATA,
    FAST_TROT,
    GAIT_STANDARD,
    MEDIUM_TROT,
    SLOW_TROT,
    MotionCmd,
    VrpnPerception,
    clamp,
    norm_angle,
)


class GoalkeeperFSM(Node):
    GO_AROUND = "go_around"
    APPROACH = "approach"
    READY = "ready"
    KICK = "kick"

    def __init__(self) -> None:
        super().__init__("goalkeeper_fsm")

        # ---- Parameters ----
        self.declare_parameter("dog_name", "mi_desktop_48_b0_2d_7b_06_3f")
        self.declare_parameter("ball_tracker", "ball")
        self.declare_parameter("dog_tracker", "LYT")
        self.declare_parameter("goal_tracker", "goal_left")
        self.declare_parameter("forward_yaw_axis", "x")
        self.declare_parameter("yaw_proj_min", 0.10)
        self.declare_parameter("yaw_filter_alpha", 0.35)

        # Goalkeeper-specific
        self.declare_parameter("goal_distance", 0.3)
        self.declare_parameter("approach_speed", 0.5)
        self.declare_parameter("arrive_tol", 0.5)
        self.declare_parameter("goal_half_width", 0.70)
        self.declare_parameter("goalkeeper_half_range", 0.4)
        self.declare_parameter("kick_duration", 0.3)
        self.declare_parameter("kick_ball_dist_thresh", 0.3)
        self.declare_parameter("close_ball_dist", 1.0)
        self.declare_parameter("close_frames_needed", 3)
        self.declare_parameter("stand_delay", 3.0)

        self.declare_parameter("goal_fallback_x", 0.0)
        self.declare_parameter("goal_fallback_y", -4.5)

        p = self.get_parameter
        self.dog_name = p("dog_name").value
        self.goal_distance = float(p("goal_distance").value)
        self.approach_speed = float(p("approach_speed").value)
        self.arrive_tol = float(p("arrive_tol").value)
        self.goal_half_width = float(p("goal_half_width").value)
        self.goalkeeper_half_range = float(p("goalkeeper_half_range").value)
        self.kick_duration = float(p("kick_duration").value)
        self.kick_ball_dist_thresh = float(p("kick_ball_dist_thresh").value)
        self.close_ball_dist = float(p("close_ball_dist").value)
        self.close_frames_needed = int(p("close_frames_needed").value)
        self.stand_delay = float(p("stand_delay").value)

        goal_fallback_x = float(p("goal_fallback_x").value)
        goal_fallback_y = float(p("goal_fallback_y").value)
        self.goal_fallback_xy = (goal_fallback_x, goal_fallback_y)

        # ---- VRPN perception ----
        self.perception = VrpnPerception(
            node=self,
            ball_tracker=p("ball_tracker").value,
            dog_tracker=p("dog_tracker").value,
            goal_tracker=p("goal_tracker").value,
            yaw_axis=str(p("forward_yaw_axis").value).lower(),
            yaw_proj_min=float(p("yaw_proj_min").value),
            yaw_alpha=clamp(float(p("yaw_filter_alpha").value), 0.0, 1.0),
        )
        self.get_logger().info(
            "Defending goal: {} (fallback: {:.1f},{:.1f})".format(
                p("goal_tracker").value,
                goal_fallback_x,
                goal_fallback_y,
            )
        )

        # ---- Motion publisher ----
        self.pub_cmd = self.create_publisher(
            MotionServoCmd,
            "/{}/motion_servo_cmd".format(self.dog_name),
            10,
        )

        # ---- Stand up ----
        self.stand_client = self.create_client(
            MotionResultCmd,
            "/{}/motion_result_cmd".format(self.dog_name),
        )
        self.get_logger().info("Standing up...")
        self.stand_client.wait_for_service(timeout_sec=5.0)
        req = MotionResultCmd.Request()
        req.motion_id = 111
        req.cmd_source = 4
        req.step_height = [0.05, 0.05]
        self.stand_client.call_async(req)

        # ---- State ----
        self.state = self.GO_AROUND
        self.start_time = time.monotonic()
        self.close_frames: int = 0
        self.kick_start: float = 0.0
        self.last_cmd = MotionCmd(SLOW_TROT, 0.0, 0.0, 0.0)
        self._last_log_t: float = 0.0

        self.create_timer(0.05, self._tick)
        self.get_logger().info("Goalkeeper FSM started.")

    def _cmd(self, cmd: MotionCmd) -> None:
        msg = MotionServoCmd()
        msg.motion_id = int(cmd.gait)
        msg.cmd_type = CMD_DATA
        msg.value = GAIT_STANDARD
        msg.vel_des = [float(cmd.vx), float(cmd.vy), float(cmd.vyaw)]
        msg.step_height = [0.05, 0.05]
        self.pub_cmd.publish(msg)
        self.last_cmd = cmd

    def _stop(self) -> None:
        self._cmd(MotionCmd(MEDIUM_TROT, 0.0, 0.0, 0.0))

    @staticmethod
    def _to_body(wx: float, wy: float, dog_yaw: float):
        cyaw = math.cos(dog_yaw)
        syaw = math.sin(dog_yaw)
        return (wx * cyaw + wy * syaw, -wx * syaw + wy * cyaw)

    def _log(self, text: str, period_s: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._last_log_t > period_s:
            self.get_logger().info(text)
            self._last_log_t = now

    def _tick(self) -> None:
        now = time.monotonic()
        if now - self.start_time < self.stand_delay:
            return

        snapshot = self.perception.snapshot(self.goal_fallback_xy)
        if snapshot is None:
            self._log("Waiting VRPN snapshot, keep stepping in place")
            self._stop()
            return

        stale = self.perception.stale_items(now, max_age=2.0)
        stale = [item for item in stale if not item.startswith("goal(")]
        if stale:
            self._log("Stale VRPN: {}".format(" ".join(stale)))
            self._stop()
            return

        s = snapshot
        dog_x, dog_y = s.dog_x, s.dog_y
        dog_yaw = s.dog_yaw
        goal_x, goal_y = s.goal_x, s.goal_y
        ball_x, ball_y = s.ball_x, s.ball_y
        ball_dist = math.hypot(ball_x - dog_x, ball_y - dog_y)

        # Field direction: from goal toward ball (ball is on the field side)
        gx = ball_x - goal_x
        gy = ball_y - goal_y
        g_dist = math.hypot(gx, gy)
        if g_dist < 1e-6:
            ux, uy = 0.0, 1.0
        else:
            ux, uy = gx / g_dist, gy / g_dist

        # Goal line direction (perpendicular to field_dir, left side)
        glx, gly = -uy, ux

        # Desired yaw: body parallel to goal line (facing into field)
        desired_yaw_align = math.atan2(uy, ux) + math.pi / 2.0

        # ---- GO_AROUND: navigate to a waypoint in front of goal ----
        if self.state == self.GO_AROUND:
            if dog_y >= goal_y + 1.0:
                self.get_logger().info("Already in front of goal, skip GO_AROUND")
                self.state = self.APPROACH
                return

            waypoint_x = dog_x
            waypoint_y = goal_y + 0.6

            wdx = waypoint_x - dog_x
            wdy = waypoint_y - dog_y
            wdist = math.hypot(wdx, wdy)

            if wdist < 0.2:
                self.get_logger().info("GO_AROUND reached waypoint, switching to APPROACH")
                self.state = self.APPROACH
                return

            wx = wdx / wdist * self.approach_speed
            wy = wdy / wdist * self.approach_speed
            vx_b, vy_b = self._to_body(wx, wy, dog_yaw)
            self._cmd(MotionCmd(SLOW_TROT, vx_b, vy_b, 0.0))
            self._log(
                "[GO_AROUND] waypoint=({:.2f},{:.2f}) dist={:.2f} bv=({:.2f},{:.2f})".format(
                    waypoint_x, waypoint_y, wdist, vx_b, vy_b
                )
            )
            return

        # ---- APPROACH: walk to 30cm in front of goal ----
        elif self.state == self.APPROACH:
            target_x = goal_x + ux * self.goal_distance
            target_y = goal_y + uy * self.goal_distance

            dx = target_x - dog_x
            dy = target_y - dog_y
            dist = math.hypot(dx, dy)

            if dist < self.arrive_tol:
                self.get_logger().info("Arrived! Switching to READY")
                self.state = self.READY
                return

            wx = dx / dist * self.approach_speed
            wy = dy / dist * self.approach_speed
            vx_b, vy_b = self._to_body(wx, wy, dog_yaw)
            self._cmd(MotionCmd(FAST_TROT, vx_b, vy_b, 0.0))
            self._log(
                "[APPROACH] dist={:.2f} bv=({:.2f},{:.2f}) yaw={:.1f}deg".format(
                    dist, vx_b, vy_b, math.degrees(dog_yaw)
                )
            )
            return

        # ---- READY: track ball along goal line, body parallel to goal line ----
        elif self.state == self.READY:
            # Count consecutive frames where ball is close to dog
            if ball_y < goal_y:
                self.close_frames = 0
            elif ball_dist < self.close_ball_dist:
                self.close_frames += 1
            else:
                self.close_frames = 0

            if self.close_frames >= self.close_frames_needed:
                self.get_logger().info("Ball blocked! Kicking away...")
                self.state = self.KICK
                self.kick_start = now
                return

            # Predict where ball crosses goal line, track that laterally
            intercept_x = self.perception.predict_intercept_x(goal_y)
            if intercept_x is not None:
                ball_proj = goal_x - intercept_x
            else:
                ball_proj = (ball_x - goal_x) * glx + (ball_y - goal_y) * gly
            ball_proj = clamp(ball_proj, -self.goal_half_width, self.goal_half_width)

            # Target Y: goal_distance from goal toward field
            # Target X: track ball projection, clamped to goalkeeper range
            rtx = goal_x + glx * ball_proj + ux * self.goal_distance
            rtx = clamp(rtx, goal_x - self.goalkeeper_half_range, goal_x + self.goalkeeper_half_range)
            rty = goal_y + gly * ball_proj + uy * self.goal_distance

            rdx = rtx - dog_x
            rdy = rty - dog_y
            rdist = math.hypot(rdx, rdy)

            yaw_err = norm_angle(desired_yaw_align - dog_yaw)
            vyaw = clamp(0.8 * yaw_err, -1.0, 1.0)

            if rdist > 0.01:
                wx = clamp(1.2 * rdx, -1.0, 1.0)
                wy = clamp(1.2 * rdy, -1.0, 1.0)
                vx_b, vy_b = self._to_body(wx, wy, dog_yaw)
                self._cmd(MotionCmd(FAST_TROT, vx_b, vy_b, vyaw))
            else:
                self._cmd(MotionCmd(FAST_TROT, 0.0, 0.0, vyaw))

            self._log(
                "[READY] intercept={} ball_proj={:.2f} target=({:.2f},{:.2f}) rdist={:.3f} yaw_err={:.1f}deg".format(
                    "({:.2f})".format(intercept_x) if intercept_x is not None else "none",
                    ball_proj, rtx, rty, rdist, math.degrees(yaw_err),
                )
            )
            return

        # ---- KICK: short dash toward ball to push it away ----
        elif self.state == self.KICK:
            if now - self.kick_start > self.kick_duration or ball_dist > self.kick_ball_dist_thresh:
                self.get_logger().info("Kick done, back to READY")
                self.state = self.READY
                self.close_frames = 0
                return

            kdx = ball_x - dog_x
            kdy = ball_y - dog_y
            kdist = math.hypot(kdx, kdy)
            if kdist > 0.01:
                wx = kdx / kdist * 1.5
                wy = kdy / kdist * 1.5
            else:
                wx, wy = 0.0, 1.5
            vx_b, vy_b = self._to_body(wx, wy, dog_yaw)
            self._cmd(MotionCmd(FAST_TROT, vx_b, vy_b, 0.0))
            self._log(
                "[KICK] ball=({:.2f},{:.2f}) dist={:.2f} bv=({:.2f},{:.2f})".format(
                    ball_x, ball_y, kdist, vx_b, vy_b
                )
            )
            return

        # Fallback: stop
        self._stop()

    def destroy_node(self) -> bool:
        self._stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoalkeeperFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
