#!/usr/bin/env python3

"""
Simple ROS 2 Service Example
This demonstrates the basic structure of a ROS 2 service server and client
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceServer(Node):
    """
    A simple service server that adds two integers
    """

    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Simple Service Server has started')

    def add_two_ints_callback(self, request, response):
        """
        Callback function that handles service requests
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response


class SimpleServiceClient(Node):
    """
    A simple service client that calls the add_two_ints service
    """

    def __init__(self):
        super().__init__('simple_service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to the service
        """
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future


def server_main(args=None):
    """
    Main function to run the service server
    """
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()

    try:
        rclpy.spin(simple_service_server)
    except KeyboardInterrupt:
        pass

    simple_service_server.destroy_node()
    rclpy.shutdown()


def client_main(args=None):
    """
    Main function to run the service client
    """
    rclpy.init(args=args)
    simple_service_client = SimpleServiceClient()

    # Send a request
    future = simple_service_client.send_request(2, 3)

    try:
        # Wait for the response
        rclpy.spin_until_future_complete(simple_service_client, future)
        response = future.result()
        if response is not None:
            print(f'Result of add_two_ints: {response.sum}')
        else:
            print('Service call failed')
    except KeyboardInterrupt:
        print('Interrupted during service call')

    simple_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'client':
        client_main()
    else:
        server_main()