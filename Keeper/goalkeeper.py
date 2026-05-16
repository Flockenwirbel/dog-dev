#!/usr/bin/env python3
"""Goalkeeper: walk to 30cm in front of goal_left using reference VRPN framework."""

import math
import subprocess
import time

import rclpy
from protocol.msg import MotionServoCmd
from protocol.srv import MotionResultCmd
from rclpy.node import Node

from .vrpn_perception import (
    SLOW_TROT,
    MEDIUM_TROT,
    CMD_DATA,
    GAIT_STANDARD,
    VrpnPerception,
    MotionCmd,
    clamp,
    norm_angle,
)


DOG_NAMESPACE = "mi_desktop_48_b0_2d_7b_06_3f"
GOAL_DISTANCE = 0.3


def _to_body(wx, wy, cyaw, syaw):
    """World→body frame: (forward, left). Same as reference approach_controller."""
    return (wx * cyaw + wy * syaw, -wx * syaw + wy * cyaw)


class Goalkeeper(Node):
    def __init__(self):
        super().__init__("goalkeeper")

        # ---- Same parameter style as reference ball_kicker ----
        self.declare_parameter("dog_name", DOG_NAMESPACE)
        self.declare_parameter("ball_tracker", "ball")
        self.declare_parameter("dog_tracker", "LYT")
        self.declare_parameter("goal_tracker", "goal_left")
        self.declare_parameter("forward_yaw_axis", "x")
        self.declare_parameter("yaw_proj_min", 0.10)
        self.declare_parameter("yaw_filter_alpha", 0.35)
        self.declare_parameter("approach_speed", 0.25)
        self.declare_parameter("yaw_gain", 0.5)
        self.declare_parameter("max_yaw_rate", 0.5)
        self.declare_parameter("goal_distance", 0.3)

        p = self.get_parameter
        self.dog_name = p("dog_name").value
        self.approach_speed = float(p("approach_speed").value)
        self.yaw_gain = float(p("yaw_gain").value)
        self.max_yaw_rate = float(p("max_yaw_rate").value)
        self.goal_distance = float(p("goal_distance").value)

        # ---- VRPN perception (same as reference) ----
        self.perception = VrpnPerception(
            node=self,
            ball_tracker=p("ball_tracker").value,
            dog_tracker=p("dog_tracker").value,
            goal_tracker=p("goal_tracker").value,
            yaw_axis=str(p("forward_yaw_axis").value).lower(),
            yaw_proj_min=float(p("yaw_proj_min").value),
            yaw_alpha=clamp(float(p("yaw_filter_alpha").value), 0.0, 1.0),
        )

        self.goal_fixed_xy = (0.0, -4.5)

        # ---- Motion publisher (same as reference) ----
        self.pub_cmd = self.create_publisher(
            MotionServoCmd,
            "/{}/motion_servo_cmd".format(self.dog_name),
            10,
        )

        # ---- Stand up first ----
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

        self.start_time = time.monotonic()
        self.create_timer(0.1, self._tick)
        self.get_logger().info("Goalkeeper started.")

    def _cmd(self, cmd):
        msg = MotionServoCmd()
        msg.motion_id = int(cmd.gait)
        msg.cmd_type = CMD_DATA
        msg.value = GAIT_STANDARD
        msg.vel_des = [float(cmd.vx), float(cmd.vy), float(cmd.vyaw)]
        msg.step_height = [0.05, 0.05]
        self.pub_cmd.publish(msg)

    def _tick(self):
        now = time.monotonic()
        if now - self.start_time < 3.0:
            return

        snapshot = self.perception.snapshot(self.goal_fixed_xy)
        if snapshot is None:
            self._log("Waiting VRPN snapshot, keep stepping in place")
            self._cmd(MotionCmd(SLOW_TROT, 0.0, 0.0, 0.0))
            return

        stale = self.perception.stale_items(now, max_age=2.0)
        stale = [item for item in stale if not item.startswith("goal(")]
        if stale:
            self._log("Stale VRPN: {}".format(" ".join(stale)))
            self._cmd(MotionCmd(SLOW_TROT, 0.0, 0.0, 0.0))
            return

        s = snapshot
        goal_x, goal_y = s.goal_x, s.goal_y
        dog_x, dog_y = s.dog_x, s.dog_y
        dog_yaw = s.dog_yaw

        # ---- Target: goal_left position + 30cm toward field ----
        # Use ball to determine which side is the field
        ball_x, ball_y = s.ball_x, s.ball_y
        gx = ball_x - goal_x
        gy = ball_y - goal_y
        g_dist = math.hypot(gx, gy)

        if g_dist < 1e-6:
            self._cmd(MotionCmd(SLOW_TROT, 0.0, 0.0, 0.0))
            return

        ux = gx / g_dist
        uy = gy / g_dist

        target_x = goal_x + ux * self.goal_distance
        target_y = goal_y + uy * self.goal_distance

        dx = target_x - dog_x
        dy = target_y - dog_y
        dist = math.hypot(dx, dy)

        if dist < 0.05:
            self.get_logger().info("On target. dist={:.3f}".format(dist))
            self._cmd(MotionCmd(SLOW_TROT, 0.0, 0.0, 0.0))
            return

        # World frame velocity toward target (same as reference approach_controller)
        speed = self.approach_speed
        if dist > 1e-6:
            tow_x = dx / dist * speed
            tow_y = dy / dist * speed
        else:
            tow_x, tow_y = 0.0, 0.0

        # World -> body (same as reference)
        cyaw = math.cos(dog_yaw)
        syaw = math.sin(dog_yaw)
        vx_body, vy_body = _to_body(tow_x, tow_y, cyaw, syaw)

        # Face toward target
        desired_yaw = math.atan2(dy, dx)
        yaw_err = norm_angle(desired_yaw - dog_yaw)
        vyaw = clamp(self.yaw_gain * yaw_err, -self.max_yaw_rate, self.max_yaw_rate)

        self._cmd(MotionCmd(SLOW_TROT, vx_body, vy_body, vyaw))

        self.get_logger().info(
            "[approach] dog=({:.2f},{:.2f}) target=({:.2f},{:.2f}) dist={:.2f} bv=({:.2f},{:.2f}) vyaw={:.2f} yaw={:.1f}".format(
                dog_x, dog_y, target_x, target_y, dist,
                vx_body, vy_body, vyaw, math.degrees(dog_yaw),
            )
        )

    def _log(self, text, period_s=2.0):
        now = time.monotonic()
        if not hasattr(self, '_last_log_t'):
            self._last_log_t = 0.0
        if now - self._last_log_t > period_s:
            self.get_logger().info(text)
            self._last_log_t = now

    def destroy_node(self):
        self._cmd(MotionCmd(SLOW_TROT, 0.0, 0.0, 0.0))
        return super().destroy_node()


def launch_vrpn():
    cmd = (
        "source /opt/ros2/cyberdog/setup.bash 2>/dev/null && "
        "cd ~/vrpn_client_ros2/src && "
        "source install/local_setup.bash && "
        "ros2 launch vrpn_listener sync_entity_state.launch"
    )
    return subprocess.Popen(
        ["bash", "-lc", cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def main(args=None):
    print("[1/3] Starting VRPN...")
    vrpn_proc = launch_vrpn()
    print("[2/3] Waiting 3s...")
    time.sleep(3)
    print("[3/3] Starting goalkeeper...")
    rclpy.init(args=args)
    node = Goalkeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        vrpn_proc.terminate()
        print("Done.")


if __name__ == "__main__":
    main()
