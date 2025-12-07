#!/usr/bin/env python3
"""
ROS 2 Service Example for Humanoid Robotics

This example demonstrates a basic service server that responds to requests
from other nodes in a humanoid robot system.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class BasicService(Node):

    def __init__(self):
        super().__init__('basic_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    basic_service = BasicService()
    rclpy.spin(basic_service)
    basic_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()