#!/usr/bin/env python3
"""Continuously publish zero position targets for all six arm joints."""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float64MultiArray


class JointZeroPositionPublisher(Node):
    def __init__(self, rate, topic):
        super().__init__('joint_zero_position_publisher')
        self.publisher = self.create_publisher(Float64MultiArray, topic, 10)
        self.command = Float64MultiArray()
        self.command.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.command.data = [0.0, 0.5, 1.0, 0.0, 0.0, 0.0]
        # self.command.data = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.timer = self.create_timer(1.0 / rate, self.publish_command)
        self.get_logger().info(
            'Publishing six zero position targets (rad) at %.1f Hz to %s'
            % (rate, self.publisher.topic_name))

    def publish_command(self):
        self.publisher.publish(self.command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rate', type=float, default=20.0,
                        help='Publish frequency in Hz (default: 20).')
    parser.add_argument('--topic', default='/manipulator_driver/joint_pos_rad_cmd',
                        help='Driver position command topic.')
    args = parser.parse_args(remove_ros_args()[1:])
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        parser.error('--rate must be finite and positive')

    rclpy.init()
    node = JointZeroPositionPublisher(args.rate, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
