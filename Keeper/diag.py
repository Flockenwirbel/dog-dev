#!/usr/bin/env python3
"""Diagnostic: print all VRPN tracker positions and dog yaw, no movement."""

import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def yaw_from_quat(q):
    r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    r01 = 2.0 * (q.x * q.y - q.w * q.z)
    r10 = 2.0 * (q.x * q.y + q.w * q.z)
    r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)
    r02 = 2.0 * (q.x * q.z + q.w * q.y)
    r12 = 2.0 * (q.y * q.z - q.w * q.x)
    return math.atan2(r10, r00), math.atan2(r11, r01), math.atan2(r12, r02)


class Diag(Node):
    def __init__(self):
        super().__init__("diag")

        self.ball = None
        self.goal_left = None
        self.goal_right = None
        self.dog = None
        self.dog_yaws = None

        self.create_subscription(PoseStamped, "/vrpn/ball/pose", self._cb_ball, 10)
        self.create_subscription(PoseStamped, "/vrpn/goal_left/pose", self._cb_gl, 10)
        self.create_subscription(PoseStamped, "/vrpn/goal_right/pose", self._cb_gr, 10)
        self.create_subscription(PoseStamped, "/vrpn/LYT/pose", self._cb_dog, 10)

        self._last_log = 0.0
        self.create_timer(0.5, self.print_state)
        self.get_logger().info("Diagnostic node started. Move dog to each corner and wait 2s.")

    def _cb_ball(self, msg):
        self.ball = (msg.pose.position.x, msg.pose.position.y)

    def _cb_gl(self, msg):
        self.goal_left = (msg.pose.position.x, msg.pose.position.y)

    def _cb_gr(self, msg):
        self.goal_right = (msg.pose.position.x, msg.pose.position.y)

    def _cb_dog(self, msg):
        self.dog = (msg.pose.position.x, msg.pose.position.y)
        self.dog_yaws = yaw_from_quat(msg.pose.orientation)

    def print_state(self):
        now = time.monotonic()
        if now - self._last_log < 1.0:
            return
        self._last_log = now

        parts = []
        if self.dog:
            parts.append("dog=({:.2f},{:.2f})".format(self.dog[0], self.dog[1]))
        else:
            parts.append("dog=NONE")

        if self.dog_yaws:
            yx, yy, yz = self.dog_yaws
            parts.append("yaw_x={:.1f} yaw_y={:.1f} yaw_z={:.1f}".format(
                math.degrees(yx), math.degrees(yy), math.degrees(yz)))

        if self.ball:
            parts.append("ball=({:.2f},{:.2f})".format(self.ball[0], self.ball[1]))
        else:
            parts.append("ball=NONE")

        if self.goal_left:
            parts.append("goal_L=({:.2f},{:.2f})".format(self.goal_left[0], self.goal_left[1]))
        else:
            parts.append("goal_L=NONE")

        if self.goal_right:
            parts.append("goal_R=({:.2f},{:.2f})".format(self.goal_right[0], self.goal_right[1]))
        else:
            parts.append("goal_R=NONE")

        if self.dog and self.goal_left:
            dx = self.goal_left[0] - self.dog[0]
            dy = self.goal_left[1] - self.dog[1]
            dist = math.hypot(dx, dy)
            angle_to_goal = math.degrees(math.atan2(dy, dx))
            parts.append("dist_to_L={:.2f} angle_to_L={:.1f}deg".format(dist, angle_to_goal))

        self.get_logger().info(" | ".join(parts))


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
    print("Starting VRPN...")
    vrpn_proc = launch_vrpn()
    time.sleep(3)
    print("VRPN started. Launching diagnostic...")
    rclpy.init(args=args)
    node = Diag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        vrpn_proc.terminate()


if __name__ == "__main__":
    main()
