#!/usr/bin/env python3
"""Standalone node: command the dog to sit/rest, then exit."""

import rclpy
from protocol.srv import MotionResultCmd
from rclpy.node import Node

DOG_NAMESPACE = "XiaoChuan_Sun"


class SitDown(Node):
    def __init__(self):
        super().__init__("sit_down")
        self.client = self.create_client(
            MotionResultCmd,
            "/{}/motion_result_cmd".format(DOG_NAMESPACE),
        )
        self._done = False
        self.create_timer(0.5, self._tick)

    def _tick(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service not available, retrying...")
            return
        if self._done:
            return
        self._done = True

        req = MotionResultCmd.Request()
        req.motion_id = 101
        req.cmd_source = 4
        req.step_height = [0.05, 0.05]
        self.client.call_async(req)
        self.get_logger().info("Sent sit command (motion_id=101)")
        self.create_timer(1.5, self._shutdown)

    def _shutdown(self):
        self.get_logger().info("Sit-down complete.")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SitDown()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Done.")


if __name__ == "__main__":
    main()
