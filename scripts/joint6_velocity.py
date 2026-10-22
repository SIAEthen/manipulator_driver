#!/usr/bin/env python3
"""Publish a six-joint velocity command with only joint 6 moving."""

import argparse
import math
import signal

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray


class Joint6VelocityPublisher(Node):
    def __init__(self, velocity, rate, topic):
        super().__init__('joint6_velocity_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, topic, 10)
        self.command = Float64MultiArray()
        self.command.data = [0.0, 0.0, 0.0, 0.0, 0.0, velocity]
        self.timer = self.create_timer(1.0 / rate, self.publish_command)
        self.get_logger().info(
            'Publishing joint 6 velocity %.4f rad/s at %.1f Hz to %s'
            % (velocity, rate, self.publisher.topic_name))

    def publish_command(self):
        self.publisher.publish(self.command)

    def stop(self):
        self.timer.cancel()
        self.command.data = [0.0] * 6
        # 重复发送零速度，给 DDS 留出发送时间。
        for _ in range(3):
            if not rclpy.ok():
                break
            self.publish_command()
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--velocity', type=float, default=0.5,
                        help='Joint 6 velocity in rad/s (default: 0.05).')
    parser.add_argument('--rate', type=float, default=20.0,
                        help='Publish frequency in Hz (default: 20).')
    parser.add_argument('--topic', default='/manipulator_driver/joint_vel_rads_cmd',
                        help='Driver velocity command topic.')
    args = parser.parse_args(remove_ros_args()[1:])

    rclpy.init()
    node = Joint6VelocityPublisher(args.velocity, args.rate, args.topic)
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    # 先退出发布循环并发送零速度，再关闭 ROS context。
    previous_handlers = {
        signum: signal.signal(signum, request_stop)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            node.stop()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
