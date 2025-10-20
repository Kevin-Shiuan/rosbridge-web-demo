#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_msgs.msg import Command


class CommandListener(Node):
    def __init__(self):
        super().__init__("command_listener")
        self.sub = self.create_subscription(Command, "/my_robot/command", self.cb, 10)

    def cb(self, msg: Command):
        if msg.command == Command.START:
            self.get_logger().info("Received: START")
        elif msg.command == Command.STOP:
            self.get_logger().info("Received: STOP")
        else:
            self.get_logger().warn(f"Unknown command: {msg.command}")


def main():
    rclpy.init()
    node = CommandListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
