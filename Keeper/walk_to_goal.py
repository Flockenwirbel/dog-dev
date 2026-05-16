#!/usr/bin/env python3
"""Goalkeeper: stand 30cm in front of goal_left, body parallel to goal line, track ball laterally."""

import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from protocol.msg import MotionServoCmd
from protocol.srv import MotionResultCmd
from rclpy.node import Node

# Gait IDs from reference
SLOW_TROT = 303
MEDIUM_TROT = 308
FAST_TROT = 305
CMD_DATA = 1
GAIT_STANDARD = 2

DOG_NAMESPACE = "mi_desktop_48_b0_2d_7b_06_3f"

GOAL_DISTANCE = 0.3    # 30cm in front of goal
KP_XY = 0.8            # position proportional gain
YAW_GAIN = 0.5
MAX_SPEED = 0.25       # within slow trot limits
MAX_YAW_RATE = 0.5


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_angle(rad):
    while rad > math.pi:
        rad -= 2.0 * math.pi
    while rad < -math.pi:
        rad += 2.0 * math.pi
    return rad


class GoalkeeperPerception:
    """VRPN subscriptions for goalkeeper: ball, dog, goal."""

    def __init__(self, node, ball_tracker, dog_tracker, goal_tracker, yaw_axis, yaw_alpha):
        self.node = node
        self.yaw_axis = yaw_axis
        self.yaw_alpha = yaw_alpha

        self.ball_xy = None
        self.goal_xy = None
        self.dog_xy = None
        self.dog_yaw = None

        self.ball_t = 0.0
        self.goal_t = 0.0
        self.dog_t = 0.0

        node.create_subscription(
            PoseStamped, "/vrpn/{}/pose".format(ball_tracker), self._on_ball, 10
        )
        node.create_subscription(
            PoseStamped, "/vrpn/{}/pose".format(goal_tracker), self._on_goal, 10
        )
        node.create_subscription(
            PoseStamped, "/vrpn/{}/pose".format(dog_tracker), self._on_dog, 10
        )

    def _on_ball(self, msg):
        self.ball_xy = (msg.pose.position.x, msg.pose.position.y)
        self.ball_t = time.monotonic()

    def _on_goal(self, msg):
        self.goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.goal_t = time.monotonic()

    def _on_dog(self, msg):
        now = time.monotonic()
        self.dog_xy = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation

        r00 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        r01 = 2.0 * (q.x * q.y - q.w * q.z)
        r10 = 2.0 * (q.x * q.y + q.w * q.z)
        r11 = 1.0 - 2.0 * (q.x * q.x + q.z * q.z)
        r02 = 2.0 * (q.x * q.z + q.w * q.y)
        r12 = 2.0 * (q.y * q.z - q.w * q.x)

        self.yaw_x = math.atan2(r10, r00)
        self.yaw_y = math.atan2(r11, r01)
        self.yaw_z = math.atan2(r12, r02)

        axis_map = {
            "x": (r00, r10, self.yaw_x),
            "y": (r01, r11, self.yaw_y),
            "z": (r02, r12, self.yaw_z),
        }
        _, _, yaw_raw = axis_map[self.yaw_axis]

        if self.dog_yaw is None:
            self.dog_yaw = yaw_raw
        else:
            d = norm_angle(yaw_raw - self.dog_yaw)
            self.dog_yaw = norm_angle(self.dog_yaw + self.yaw_alpha * d)

        self.dog_t = now

    def ready(self):
        return (self.ball_xy is not None
                and self.goal_xy is not None
                and self.dog_xy is not None
                and self.dog_yaw is not None)

    def stale(self, now, max_age=2.0):
        items = []
        if now - self.ball_t > max_age:
            items.append("ball")
        if now - self.goal_t > max_age:
            items.append("goal")
        if now - self.dog_t > max_age:
            items.append("dog")
        return items


class Goalkeeper(Node):
    def __init__(self):
        super().__init__("goalkeeper")

        self.perception = GoalkeeperPerception(
            node=self,
            ball_tracker="ball",
            dog_tracker="LYT",
            goal_tracker="goal_left",
            yaw_axis="x",
            yaw_alpha=0.5,
        )

        self.cmd_pub = self.create_publisher(
            MotionServoCmd,
            "/{}/motion_servo_cmd".format(DOG_NAMESPACE),
            10
        )
        self.stand_client = self.create_client(
            MotionResultCmd,
            "/{}/motion_result_cmd".format(DOG_NAMESPACE)
        )

        # Stand up
        self.get_logger().info("Standing up...")
        self.stand_client.wait_for_service(timeout_sec=5.0)
        req = MotionResultCmd.Request()
        req.motion_id = 111
        req.cmd_source = 4
        req.step_height = [0.05, 0.05]
        self.stand_client.call_async(req)

        self.start_time = time.monotonic()
        self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        now = time.monotonic()
        if now - self.start_time < 3.0:
            return

        p = self.perception
        if not p.ready():
            self.get_logger().warn("Waiting for VRPN data...")
            return

        stale = p.stale(now)
        if stale:
            self.get_logger().warn("Stale: {}".format(" ".join(stale)))
            self._send_cmd(0.0, 0.0, 0.0)
            return

        dog_x, dog_y = p.dog_xy
        dog_yaw = p.dog_yaw
        goal_x, goal_y = p.goal_xy

        # Simple: walk to goal_left position
        target_x = goal_x
        target_y = goal_y

        dx = target_x - dog_x
        dy = target_y - dog_y
        dist = math.hypot(dx, dy)

        if dist < 0.05:
            self.get_logger().info("Arrived at goal_left! dist={:.3f}".format(dist))
            self._send_cmd(0.0, 0.0, 0.0)
            return

        # World frame P control
        world_vx = clamp(KP_XY * dx, -MAX_SPEED, MAX_SPEED)
        world_vy = clamp(KP_XY * dy, -MAX_SPEED, MAX_SPEED)

        # World -> body
        cyaw = math.cos(dog_yaw)
        syaw = math.sin(dog_yaw)
        vx_body = world_vx * cyaw + world_vy * syaw
        vy_body = -(- world_vx * syaw + world_vy * cyaw)  # negate for platform

        # Face toward target
        desired_yaw = math.atan2(dy, dx)
        yaw_err = norm_angle(desired_yaw - dog_yaw)
        vyaw = clamp(YAW_GAIN * yaw_err, -MAX_YAW_RATE, MAX_YAW_RATE)

        self._send_cmd(vx_body, vy_body, vyaw)
        self.get_logger().info(
            "dog=({:.2f},{:.2f}) goal=({:.2f},{:.2f}) dist={:.2f} bv=({:.2f},{:.2f}) vyaw={:.2f} yaw={:.1f}".format(
                dog_x, dog_y, goal_x, goal_y, dist, vx_body, vy_body, vyaw, math.degrees(dog_yaw)
            )
        )

        dx = target_x - dog_x
        dy = target_y - dog_y
        dist = math.hypot(dx, dy)

        if dist < 0.05:
            self.get_logger().info("Arrived at goal! dist={:.3f}".format(dist))
            self._send_cmd(0.0, 0.0, 0.0)
            return

        # World frame velocity (P control)
        world_vx = clamp(KP_XY * dx, -MAX_SPEED, MAX_SPEED)
        world_vy = clamp(KP_XY * dy, -MAX_SPEED, MAX_SPEED)

        # World -> body frame (same as reference approach_controller._to_body)
        cyaw = math.cos(dog_yaw)
        syaw = math.sin(dog_yaw)
        vx_body = world_vx * cyaw + world_vy * syaw
        vy_body = -world_vx * syaw + world_vy * cyaw

        # Face toward target
        desired_yaw = math.atan2(dy, dx)
        yaw_err = norm_angle(desired_yaw - dog_yaw)
        vyaw = clamp(YAW_GAIN * yaw_err, -MAX_YAW_RATE, MAX_YAW_RATE)

        self._send_cmd(vx_body, vy_body, vyaw)

        self.get_logger().info(
            "dog=({:.2f},{:.2f}) goal=({:.2f},{:.2f}) dist={:.2f} bv=({:.2f},{:.2f}) vyaw={:.2f} yaw={:.1f}deg".format(
                dog_x, dog_y, goal_x, goal_y, dist, vx_body, vy_body, vyaw, math.degrees(dog_yaw)
            )
        )

    def _send_cmd(self, vx, vy, vyaw):
        msg = MotionServoCmd()
        msg.motion_id = SLOW_TROT
        msg.cmd_type = CMD_DATA
        msg.value = GAIT_STANDARD
        msg.vel_des = [float(vx), float(vy), float(vyaw)]
        msg.step_height = [0.05, 0.05]
        self.cmd_pub.publish(msg)


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
    print("[1/3] Starting VRPN listener...")
    vrpn_proc = launch_vrpn()
    print("[2/3] Waiting 3s for VRPN...")
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
