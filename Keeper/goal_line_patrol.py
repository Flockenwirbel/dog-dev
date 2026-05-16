#!/usr/bin/env python3
"""Goalkeeper: stand 30cm in front of goal, body parallel to goal line, track ball along X."""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from protocol.msg import MotionServoCmd
from protocol.srv import MotionResultCmd


DOG_NAMESPACE = "mi_desktop_48_b0_2d_7b_06_3f"

GOAL_HALF_WIDTH = 0.75    # meters, goal total width ~1.5m
GOAL_DISTANCE = 0.3       # meters, dog stays 30cm from goal
KP = 0.8                  # proportional gain for position
YAW_GAIN = 0.5
MAX_SPEED = 0.25          # m/s, within slow walk limits (vx=0.65, vy=0.3)
MAX_YAW_RATE = 0.5        # rad/s, within slow walk limit (1.25)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_angle(rad):
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class Goalkeeper(Node):
    def __init__(self):
        super().__init__("goalkeeper")

        # VRPN state
        self.ball_xy = None
        self.goal_xy = None
        self.dog_xy = None
        self.dog_yaw = None
        self.ball_t = 0.0
        self.goal_t = 0.0
        self.dog_t = 0.0
        self.field_sign = None  # +1 if field in +Y, -1 if in -Y

        # VRPN subscriptions
        self.create_subscription(
            PoseStamped, "/vrpn/ball/pose", self._on_ball, 10
        )
        self.create_subscription(
            PoseStamped, "/vrpn/goal_right/pose", self._on_goal, 10
        )
        self.create_subscription(
            PoseStamped, "/vrpn/LYT/pose", self._on_dog, 10
        )

        # Motion servo command publisher
        self.cmd_pub = self.create_publisher(
            MotionServoCmd,
            "/{}/motion_servo_cmd".format(DOG_NAMESPACE),
            10
        )

        # Stand-up service client
        self.stand_client = self.create_client(
            MotionResultCmd,
            "/{}/motion_result_cmd".format(DOG_NAMESPACE)
        )

        self.get_logger().info("Waiting for stand-up service...")
        self.stand_client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("Sending stand command...")
        self._stand_up()

        self.start_time = time.monotonic()
        self.create_timer(0.1, self.control_loop)

    def _stand_up(self):
        req = MotionResultCmd.Request()
        req.motion_id = 111
        req.cmd_source = 4
        req.step_height = [0.05, 0.05]
        self.stand_client.call_async(req)

    def _on_ball(self, msg):
        self.ball_xy = (msg.pose.position.x, msg.pose.position.y)
        self.ball_t = time.monotonic()

    def _on_goal(self, msg):
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.goal_t = time.monotonic()

    def _on_dog(self, msg):
        self.dog_xy = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.dog_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.dog_t = time.monotonic()

    def control_loop(self):
        now = time.monotonic()
        if now - self.start_time < 3.0:
            return

        if self.dog_xy is None or self.dog_yaw is None:
            self.get_logger().warn("Waiting for dog VRPN data...")
            return
        if self.goal_xy is None:
            self.get_logger().warn("Waiting for goal VRPN data...")
            return
        if now - self.dog_t > 2.0:
            self.get_logger().warn("Dog data stale, stopping.")
            self._send_servo(0.0, 0.0, 0.0)
            return

        dog_x, dog_y = self.dog_xy
        dog_yaw = self.dog_yaw
        goal_x, goal_y = self.goal_xy

        # Determine field direction: ball is on the field side of the goal
        if self.ball_xy is not None and now - self.ball_t < 5.0:
            ball_x, ball_y = self.ball_xy
            if ball_y >= goal_y:
                self.field_sign = 1
            else:
                self.field_sign = -1
        if self.field_sign is None:
            self.field_sign = 1

        # Target X: track ball, clamped to goal range
        if self.ball_xy is not None and now - self.ball_t < 2.0:
            target_x = clamp(self.ball_xy[0],
                             goal_x - GOAL_HALF_WIDTH,
                             goal_x + GOAL_HALF_WIDTH)
        else:
            target_x = goal_x

        # Target Y: 30cm from goal on the field side
        target_y = goal_y + self.field_sign * GOAL_DISTANCE

        # Desired yaw: face the field (perpendicular to goal line)
        # +pi/2 means facing +Y, -pi/2 means facing -Y
        desired_yaw = (math.pi / 2.0) * self.field_sign

        # Position errors in world frame
        dx = target_x - dog_x
        dy = target_y - dog_y

        # World-frame velocities (P control)
        world_vx = KP * dx
        world_vy = KP * dy

        # Clamp world velocities
        world_vx = clamp(world_vx, -MAX_SPEED, MAX_SPEED)
        world_vy = clamp(world_vy, -MAX_SPEED, MAX_SPEED)

        # Transform to body frame
        cos_yaw = math.cos(dog_yaw)
        sin_yaw = math.sin(dog_yaw)
        body_vx = world_vx * cos_yaw + world_vy * sin_yaw
        body_vy = -world_vx * sin_yaw + world_vy * cos_yaw

        # Yaw control
        yaw_error = norm_angle(desired_yaw - dog_yaw)
        vyaw = clamp(YAW_GAIN * yaw_error, -MAX_YAW_RATE, MAX_YAW_RATE)

        self._send_servo(body_vx, body_vy, vyaw)

        self.get_logger().info(
            "target=({:.2f},{:.2f}) dog=({:.2f},{:.2f}) dx={:.2f} dy={:.2f} bv=({:.2f},{:.2f}) vyaw={:.2f}".format(
                target_x, target_y, dog_x, dog_y, dx, dy, body_vx, body_vy, vyaw
            )
        )

    def _send_servo(self, vx, vy, vyaw):
        cmd = MotionServoCmd()
        cmd.motion_id = 303
        cmd.cmd_type = 1
        cmd.cmd_source = 4
        cmd.value = 2
        cmd.vel_des = [float(vx), float(vy), float(vyaw)]
        cmd.step_height = [0.05, 0.05]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Goalkeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
