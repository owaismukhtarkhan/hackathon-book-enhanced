#!/usr/bin/env python3

"""
Simple ROS 2 Subscriber Example
This demonstrates the basic structure of a ROS 2 subscriber node
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    A simple subscriber node that receives messages from a topic
    """

    def __init__(self):
        # Initialize the node with name 'simple_subscriber'
        super().__init__('simple_subscriber')

        # Create a subscription to String messages on the 'chatter' topic
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10  # QoS history depth
        )

        # Prevent unused variable warning
        self.subscription  # type: ignore

        # Log that the subscriber has started
        self.get_logger().info('Simple Subscriber has started')

    def listener_callback(self, msg):
        """
        Callback function that gets called when a message is received
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    Main function to run the subscriber node
    """
    # Initialize the ROS 2 communication
    rclpy.init(args=args)

    # Create the subscriber node
    simple_subscriber = SimpleSubscriber()

    # Start spinning to process callbacks
    rclpy.spin(simple_subscriber)

    # Destroy the node explicitly (optional)
    simple_subscriber.destroy_node()

    # Shutdown the ROS 2 communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()