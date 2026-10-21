#!/usr/bin/env python3
"""Open and close the gripper to calibrate its encoder endpoints."""

import argparse
import math
import time

import rclpy
from rclpy.utilities import remove_ros_args
from std_srvs.srv import Trigger


def calibrate_gripper(node, driver_node, settle_time):
    steps = (
        ('open_gripper', settle_time),
        ('set_gripper_open_position', 0.0),
        ('close_gripper', settle_time),
        ('set_gripper_close_position', 0.0),
    )
    for service, delay in steps:
        service_name = driver_node.rstrip('/') + '/' + service
        client = node.create_client(Trigger, service_name)
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('Service unavailable: ' + service_name)
            node.get_logger().info('Calling ' + service_name)
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            if not future.done():
                raise RuntimeError('Service timed out: ' + service_name)
            response = future.result()
            if response is None or not response.success:
                message = response.message if response is not None else 'No response'
                raise RuntimeError(service_name + ': ' + message)
            if delay:
                node.get_logger().info('Waiting %.1f seconds for motion.' % delay)
                time.sleep(delay)
        finally:
            node.destroy_client(client)
    node.get_logger().info('Gripper open and closed positions calibrated.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--driver-node', default='/manipulator_driver',
                        help='Driver node path (default: /manipulator_driver).')
    parser.add_argument('--settle-time', type=float, default=3.0,
                        help='Seconds to wait after each motion (default: 3).')
    args = parser.parse_args(remove_ros_args()[1:])
    if not math.isfinite(args.settle_time) or args.settle_time <= 0.0:
        parser.error('--settle-time must be a finite positive number')

    rclpy.init()
    node = rclpy.create_node('gripper_calibration_helper')
    try:
        calibrate_gripper(node, args.driver_node, args.settle_time)
        return 0
    except (RuntimeError, KeyboardInterrupt) as error:
        node.get_logger().error(str(error) or 'Calibration interrupted.')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
