#!/usr/bin/env python3
"""Goalkeeper main node 鈥?ties VRPN perception to the goalkeeper controller
and sends MotionServoCmd messages to the robot."""

import math
import time

import rclpy
from rclpy.node import Node

from protocol.msg import MotionServoCmd
from protocol.srv import MotionResultCmd

from demo_python_pkg.vrpn_perception import (
    MEDIUM_TROT,
    SLOW_TROT,
    FAST_TROT,
    VrpnPerception,
)
from demo_python_pkg.goalkeeper_controller import GoalkeeperController

# cmd_source values (not all exposed as constants in generated code)
CMD_SOURCE_ALGO = 4


class GoalkeeperNode(Node):
    def __init__(self):
        super().__init__("goalkeeper_node")

        # ---- parameters (all overridable at launch time) ----
        self.declare_parameter("ball_tracker", "ball")
        self.declare_parameter("dog_tracker", "LYT")
        self.declare_parameter("goal_tracker", "goal_right")
        self.declare_parameter("yaw_axis", "z")
        self.declare_parameter("yaw_proj_min", 0.1)
        self.declare_parameter("yaw_offset", 0.0)
        self.declare_parameter("yaw_alpha", 0.5)
        self.declare_parameter("vel_smooth", 0.3)
        self.declare_parameter("goal_fallback_x", 0.0)
        self.declare_parameter("goal_fallback_y", 0.0)
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("stale_max_age", 2.0)
        self.declare_parameter("gait_guard", SLOW_TROT)
        self.declare_parameter("gait_intercept", FAST_TROT)
        self.declare_parameter("gait_push", FAST_TROT)
        self.declare_parameter("step_height", 0.05)
        self.declare_parameter("guard_dist", 0.7)
        self.declare_parameter("max_guard_dist", 1.0)
        self.declare_parameter("intercept_trigger", 2.0)
        self.declare_parameter("contact_dist", 0.4)
        self.declare_parameter("push_speed", 1.5)
        self.declare_parameter("push_angle_deg", 120.0)
        self.declare_parameter("push_duration", 0.5)
        self.declare_parameter("track_speed", 0.8)
        self.declare_parameter("lateral_speed", 0.5)
        self.declare_parameter("yaw_gain", 0.6)
        self.declare_parameter("max_yaw_rate", 1.0)

        # ---- perception ----
        self.perception = VrpnPerception(
            node=self,
            ball_tracker=self.get_parameter("ball_tracker").value,
            dog_tracker=self.get_parameter("dog_tracker").value,
            goal_tracker=self.get_parameter("goal_tracker").value,
            yaw_axis=self.get_parameter("yaw_axis").value,
            yaw_proj_min=self.get_parameter("yaw_proj_min").value,
            yaw_alpha=self.get_parameter("yaw_alpha").value,
            vel_smooth=self.get_parameter("vel_smooth").value,
            yaw_offset=self.get_parameter("yaw_offset").value,
        )

        # ---- controller ----
        self.controller = GoalkeeperController(
            guard_dist=self.get_parameter("guard_dist").value,
            max_guard_dist=self.get_parameter("max_guard_dist").value,
            intercept_trigger=self.get_parameter("intercept_trigger").value,
            contact_dist=self.get_parameter("contact_dist").value,
            push_speed=self.get_parameter("push_speed").value,
            push_angle_deg=self.get_parameter("push_angle_deg").value,
            push_duration=self.get_parameter("push_duration").value,
            track_speed=self.get_parameter("track_speed").value,
            lateral_speed=self.get_parameter("lateral_speed").value,
            yaw_gain=self.get_parameter("yaw_gain").value,
            max_yaw_rate=self.get_parameter("max_yaw_rate").value,
            gait_guard=self.get_parameter("gait_guard").value,
            gait_intercept=self.get_parameter("gait_intercept").value,
            gait_push=self.get_parameter("gait_push").value,
        )

        # ---- service client for motion commands ----
        self.motion_client = self.create_client(
            MotionResultCmd,
            "/mi_desktop_48_b0_2d_7b_06_3f/motion_result_cmd"
        )

        # ---- publisher ----
        self.cmd_pub = self.create_publisher(
            MotionServoCmd, "motion_servo_cmd", 10
        )

        # ---- control timer ----
        period = 1.0 / self.get_parameter("control_hz").value
        self.timer = self.create_timer(period, self.control_loop)

        # ---- state tracking ----
        self._last_stale_warn: float = 0.0
        self._last_state: int = -1
        self._keep_alive_timer: float = 0.0
        self._keep_alive_interval: float = 0.5
        self._active: bool = False

        self.get_logger().info(
            "GoalkeeperNode started 鈥?"
            "ball={}, dog={}, goal={}, hz={}".format(
                self.get_parameter("ball_tracker").value,
                self.get_parameter("dog_tracker").value,
                self.get_parameter("goal_tracker").value,
                self.get_parameter("control_hz").value,
            )
        )

        # Send a keep-alive DATA frame immediately so the robot enters
        # servo mode.
        self._send_servo_start()
        self.get_logger().info("Calling motion_id=201 (stand)...")
        self._call_motion(201, 5000)
        self.get_logger().info("Stand command done")

    # ------------------------------------------------------------------
    #  Main control loop
    # ------------------------------------------------------------------
    def control_loop(self):
        now = self.perception.node.get_clock().now().nanoseconds * 1e-9

        # Periodic keep-alive
        if now - self._keep_alive_timer > self._keep_alive_interval:
            self._send_keep_alive()
            self._keep_alive_timer = now

        # Check stale trackers
        stale = self.perception.stale_items(
            time.monotonic(),
            max_age=self.get_parameter("stale_max_age").value,
        )
        if stale:
            self.get_logger().warn(
                "Stale trackers: {}".format(", ".join(stale)),
                throttle_duration_sec=2.0,
            )

        # Build snapshot
        goal_fb_x = self.get_parameter("goal_fallback_x").value
        goal_fb_y = self.get_parameter("goal_fallback_y").value
        snap = self.perception.snapshot(
            goal_fallback_xy=(goal_fb_x, goal_fb_y)
        )
        if snap is None:
            self.get_logger().warn(
                "Perception not ready", throttle_duration_sec=2.0
            )
            return

        self._active = True

        # Build inputs
        dog_pos = (snap.dog_x, snap.dog_y, snap.dog_yaw)
        ball_pos = (snap.ball_x, snap.ball_y)
        ball_vel = (snap.ball_vx, snap.ball_vy)
        goal_pos = (snap.goal_x, snap.goal_y)

        # Run controller step
        motion_cmd = self.controller.step(
            dog_pos, ball_pos, ball_vel, goal_pos, snap.t
        )

        # Log state transitions
        if self.controller.state != self._last_state:
            self.get_logger().info(
                "STATE 鈫?{}".format(self.controller.state_name)
            )
            self._last_state = self.controller.state

        # Publish
        self._publish_cmd(motion_cmd)

    # ------------------------------------------------------------------
    #  Publish a single MotionServoCmd
    # ------------------------------------------------------------------
    def _publish_cmd(self, motion_cmd) -> None:
        msg = MotionServoCmd()
        msg.motion_id = motion_cmd.gait
        msg.cmd_type = MotionServoCmd.SERVO_DATA
        msg.cmd_source = CMD_SOURCE_ALGO
        msg.value = 0
        msg.vel_des = [
            float(motion_cmd.vx),
            float(motion_cmd.vy),
            float(motion_cmd.vyaw),
        ]
        msg.rpy_des = [0.0, 0.0, 0.0]
        msg.pos_des = [0.0, 0.0, 0.0]
        msg.acc_des = [0.0, 0.0, 0.0]
        msg.ctrl_point = [0.0, 0.0, 0.0]
        msg.foot_pose = [0.0, 0.0, 0.0]
        msg.step_height = [
            float(self.get_parameter("step_height").value),
            float(self.get_parameter("step_height").value),
        ]
        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    #  Keep-alive 鈥?send an empty DATA frame so the robot knows we are
    #  alive (prevents it from timing out and stopping).
    # ------------------------------------------------------------------
    def _call_motion(self, motion_id: int, duration: int = 1000) -> bool:
        req = MotionResultCmd.Request()
        req.motion_id = motion_id
        req.cmd_source = 4
        req.vel_des = [0.0, 0.0, 0.0]
        req.rpy_des = [0.0, 0.0, 0.0]
        req.pos_des = [0.0, 0.0, 0.0]
        req.acc_des = [0.0, 0.0, 0.0]
        req.ctrl_point = [0.0, 0.0, 0.0]
        req.foot_pose = [0.0, 0.0, 0.0]
        req.step_height = [0.05, 0.05]
        req.duration = duration
        req.value = 0
        req.contact = 0
        req.gait_toml = ""
        req.toml_data = ""
        future = self.motion_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            resp = future.result()
            if resp.result:
                self.get_logger().info(
                    "Motion {} OK -> {}".format(motion_id, resp.motion_id)
                )
                return True
            else:
                self.get_logger().warn(
                    "Motion {} failed code={}".format(motion_id, resp.code)
                )
        return False

    def _send_servo_start(self) -> None:
        """Send SERVO_START to enter servo mode (required before DATA)."""
        msg = MotionServoCmd()
        msg.motion_id = self.get_parameter("gait_guard").value
        msg.cmd_type = MotionServoCmd.SERVO_START
        msg.cmd_source = CMD_SOURCE_ALGO
        msg.value = 0
        msg.vel_des = [0.0, 0.0, 0.0]
        msg.rpy_des = [0.0, 0.0, 0.0]
        msg.pos_des = [0.0, 0.0, 0.0]
        msg.acc_des = [0.0, 0.0, 0.0]
        msg.ctrl_point = [0.0, 0.0, 0.0]
        msg.foot_pose = [0.0, 0.0, 0.0]
        msg.step_height = [
            float(self.get_parameter("step_height").value),
            float(self.get_parameter("step_height").value),
        ]
        self.cmd_pub.publish(msg)
        self.get_logger().info("Sent SERVO_START")

    def _send_keep_alive(self) -> None:
        if False:  # always keep alive
            return
        # Send a zero-velocity command 鈥?same as publishing an IDLE
        # command.  The next real control-loop tick will overwrite it.
        msg = MotionServoCmd()
        msg.motion_id = self.get_parameter("gait_guard").value
        msg.cmd_type = MotionServoCmd.SERVO_DATA
        msg.cmd_source = CMD_SOURCE_ALGO
        msg.value = 0
        msg.vel_des = [0.0, 0.0, 0.0]
        msg.rpy_des = [0.0, 0.0, 0.0]
        msg.pos_des = [0.0, 0.0, 0.0]
        msg.acc_des = [0.0, 0.0, 0.0]
        msg.ctrl_point = [0.0, 0.0, 0.0]
        msg.foot_pose = [0.0, 0.0, 0.0]
        msg.step_height = [
            float(self.get_parameter("step_height").value),
            float(self.get_parameter("step_height").value),
        ]
        self.cmd_pub.publish(msg)

    def destroy_node(self) -> None:
        # Send SERVO_END to release the robot from servo mode
        if self._active:
            msg = MotionServoCmd()
            msg.motion_id = SLOW_TROT
            msg.cmd_type = MotionServoCmd.SERVO_END
            msg.cmd_source = CMD_SOURCE_ALGO
            msg.value = 0
            msg.vel_des = [0.0, 0.0, 0.0]
            msg.rpy_des = [0.0, 0.0, 0.0]
            msg.pos_des = [0.0, 0.0, 0.0]
            msg.acc_des = [0.0, 0.0, 0.0]
            msg.ctrl_point = [0.0, 0.0, 0.0]
            msg.foot_pose = [0.0, 0.0, 0.0]
            msg.step_height = [0.05, 0.05]
            self.cmd_pub.publish(msg)
            self.get_logger().info("Sent SERVO_END")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoalkeeperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
