#!/usr/bin/env python3
"""Command the dog to move forward at maximum speed.

Usage (on Dog after deploy):
    source /etc/mi/ros2_env.conf
    source ~/ros2_ws/install/local_setup.bash
    python3 ~/ros2_ws/src/demo_python_pkg/demo_python_pkg/max_forward.py
"""

import rclpy
from protocol.msg import MotionServoCmd
from rclpy.node import Node

FAST_TROT = 305
CMD_DATA = 1
GAIT_STANDARD = 2


class MaxForward(Node):
    def __init__(self):
        super().__init__("max_forward")

        self.declare_parameter("dog_name", "XiaoChuan_Sun")
        self.declare_parameter("speed", 3.0)

        dog_name = self.get_parameter("dog_name").value
        self.speed = float(self.get_parameter("speed").value)

        self.pub = self.create_publisher(
            MotionServoCmd,
            "/{}/motion_servo_cmd".format(dog_name),
            10,
        )

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            "MaxForward: dog={} speed={:.1f} m/s".format(dog_name, self.speed)
        )

    def _tick(self):
        msg = MotionServoCmd()
        msg.motion_id = FAST_TROT
        msg.cmd_type = CMD_DATA
        msg.value = GAIT_STANDARD
        msg.vel_des = [self.speed, 0.0, 0.0]
        msg.step_height = [0.05, 0.05]
        self.pub.publish(msg)

    def destroy_node(self):
        msg = MotionServoCmd()
        msg.motion_id = FAST_TROT
        msg.cmd_type = CMD_DATA
        msg.value = GAIT_STANDARD
        msg.vel_des = [0.0, 0.0, 0.0]
        msg.step_height = [0.05, 0.05]
        self.pub.publish(msg)
        self.get_logger().info("Stop sent")
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MaxForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
