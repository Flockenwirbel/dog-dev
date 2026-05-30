#!/usr/bin/env python3
"""Goalkeeper: approach goal_left at 2m/s, then turn sideways (body parallel to goal line)."""

import math
import subprocess
import time

import rclpy
from protocol.msg import MotionServoCmd
from protocol.srv import MotionResultCmd
from rclpy.node import Node

from .vrpn_perception import (
    CMD_DATA,
    FAST_TROT,
    GAIT_STANDARD,
    SLOW_TROT,
    VrpnPerception,
    clamp,
    norm_angle,
)

DOG_NAMESPACE = "mi_desktop_48_b0_2d_7b_06_3f"
GOAL_DISTANCE = 0.3
APPROACH_SPEED = 1.0
ARRIVE_TOL = 0.5
GOAL_HALF_WIDTH = 0.70
GOALKEEPER_HALF_RANGE = 0.5


class Goalkeeper(Node):
    GO_AROUND = "go_around"
    APPROACH = "approach"
    READY = "ready"

    def __init__(self):
        super().__init__("goalkeeper")

        self.declare_parameter("goal_tracker", "goal_left")
        goal_tracker = self.get_parameter("goal_tracker").value
        self.field_sign = 1 if goal_tracker == "goal_left" else -1
        self.get_logger().info("Field sign: {} (+Y)".format(self.field_sign))

        self.perception = VrpnPerception(
            node=self, ball_tracker="ball", dog_tracker="dog_3_2",
            goal_tracker=goal_tracker, yaw_axis="x", yaw_alpha=0.35,
        )
        self.get_logger().info("Defending goal: {}".format(goal_tracker))
        self.cmd_pub = self.create_publisher(
            MotionServoCmd, "/{}/motion_servo_cmd".format(DOG_NAMESPACE), 10
        )
        self.stand_client = self.create_client(
            MotionResultCmd, "/{}/motion_result_cmd".format(DOG_NAMESPACE)
        )

        self.get_logger().info("Standing up...")
        self.stand_client.wait_for_service(timeout_sec=5.0)
        req = MotionResultCmd.Request()
        req.motion_id = 111
        req.cmd_source = 4
        req.step_height = [0.05, 0.05]
        self.stand_client.call_async(req)

        self.state = self.GO_AROUND
        self.create_timer(0.05, self._tick)

    def _send(self, gait, vx, vy, vyaw):
        msg = MotionServoCmd()
        msg.motion_id = gait
        msg.cmd_type = CMD_DATA
        msg.value = GAIT_STANDARD
        msg.vel_des = [float(vx), float(vy), float(vyaw)]
        msg.step_height = [0.05, 0.05]
        self.cmd_pub.publish(msg)

    def _to_body(self, wx, wy, dog_yaw):
        cyaw = math.cos(dog_yaw)
        syaw = math.sin(dog_yaw)
        return wx * cyaw + wy * syaw, -wx * syaw + wy * cyaw

    def _tick(self):
        now = time.monotonic()

        p = self.perception
        if not p.ready():
            self.get_logger().warn("Waiting VRPN...")
            return
        stale = p.stale(now)
        if stale:
            self.get_logger().warn("Stale: {}".format(" ".join(stale)))
            self._send(SLOW_TROT, 0.0, 0.0, 0.0)
            return

        dog_x, dog_y = p.dog_xy
        dog_yaw = p.dog_yaw
        goal_x, goal_y = p.goal_xy
        ball_x, ball_y = p.ball_xy

        # Field direction
        ux, uy = 0.0, self.field_sign

        # Goal line direction (perpendicular to field_dir)
        glx, gly = -uy, ux

        # Target: 30cm from goal toward field (center)
        target_x = goal_x + ux * GOAL_DISTANCE
        target_y = goal_y + uy * GOAL_DISTANCE

        # Errors in field_dir frame
        dx = target_x - dog_x
        dy = target_y - dog_y
        dist = math.hypot(dx, dy)

        # Desired yaw: body parallel to goal line
        desired_yaw_align = math.atan2(uy, ux) + math.pi / 2.0

        if self.state == self.GO_AROUND:
            if dog_y * self.field_sign >= goal_y * self.field_sign + 1.0:
                self.get_logger().info("Already in front of goal, skip GO_AROUND")
                self.state = self.APPROACH
                return

            waypoint_x = dog_x
            waypoint_y = goal_y + self.field_sign * 0.6

            wdx = waypoint_x - dog_x
            wdy = waypoint_y - dog_y
            wdist = math.hypot(wdx, wdy)

            if wdist < 0.2:
                self.get_logger().info("GO_AROUND reached waypoint, switching to APPROACH")
                self.state = self.APPROACH
                return

            wx = wdx / wdist * APPROACH_SPEED
            wy = wdy / wdist * APPROACH_SPEED
            vx_b, vy_b = self._to_body(wx, wy, dog_yaw)
            self._send(SLOW_TROT, vx_b, vy_b, 0.0)
            self.get_logger().info(
                "[GO_AROUND] waypoint=({:.2f},{:.2f}) dist={:.2f} bv=({:.2f},{:.2f})".format(
                    waypoint_x, waypoint_y, wdist, vx_b, vy_b))
            return

        elif self.state == self.APPROACH:
            if dist < ARRIVE_TOL:
                self.get_logger().info("Arrived! Switching to READY")
                self.state = self.READY
                return
            wx = dx / dist * APPROACH_SPEED
            wy = dy / dist * APPROACH_SPEED
            vx_b, vy_b = self._to_body(wx, wy, dog_yaw)
            self._send(FAST_TROT, vx_b, vy_b, 0.0)
            self.get_logger().info(
                "[APPROACH] dist={:.2f} bv=({:.2f},{:.2f}) yaw={:.1f}".format(
                    dist, vx_b, vy_b, math.degrees(dog_yaw)))

        elif self.state == self.READY:
            intercept_x = p.predict_intercept_x(goal_y, self.field_sign)
            if intercept_x is not None:
                ball_proj = (goal_x - intercept_x) * self.field_sign
            else:
                ball_proj = (ball_x - goal_x) * glx + (ball_y - goal_y) * gly
            ball_proj = clamp(ball_proj, -GOAL_HALF_WIDTH, GOAL_HALF_WIDTH)

            rtx = goal_x + glx * ball_proj + ux * GOAL_DISTANCE
            rtx = clamp(rtx, goal_x - GOALKEEPER_HALF_RANGE, goal_x + GOALKEEPER_HALF_RANGE)
            rty = goal_y + gly * ball_proj + uy * GOAL_DISTANCE
            rdx = rtx - dog_x
            rdy = rty - dog_y
            rdist = math.hypot(rdx, rdy)

            yaw_err = norm_angle(desired_yaw_align - dog_yaw)
            vyaw = clamp(0.8 * yaw_err, -1.0, 1.0)

            if rdist > 0.01:
                wx = clamp(1.5 * rdx, -1.2, 1.2)
                wy = clamp(1.5 * rdy, -1.2, 1.2)
                if dog_x >= goal_x + GOALKEEPER_HALF_RANGE:
                    wx = min(0.0, wx)
                if dog_x <= goal_x - GOALKEEPER_HALF_RANGE:
                    wx = max(0.0, wx)
                vx_b, vy_b = self._to_body(wx, wy, dog_yaw)
                self._send(FAST_TROT, vx_b, vy_b, vyaw)
            else:
                self._send(FAST_TROT, 0.0, 0.0, vyaw)

            self.get_logger().info(
                "[READY] intercept={} ball_proj={:.2f} target=({:.2f},{:.2f}) rdist={:.3f} yaw_err={:.1f}deg".format(
                    "({:.2f})".format(intercept_x) if intercept_x is not None else "none",
                    ball_proj, rtx, rty, rdist, math.degrees(yaw_err)))

    def destroy_node(self):
        self._send(SLOW_TROT, 0.0, 0.0, 0.0)
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
    import sys
    main(args=sys.argv)
