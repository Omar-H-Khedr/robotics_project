import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleListener(Node):
    """A small node that listens for robot status messages."""

    def __init__(self):
        super().__init__('simple_listener')

        # Subscribe to the same topic used by simple_talker.
        self.subscription = self.create_subscription(
            String,
            '/robot_status',
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg):
        """Print each message received from /robot_status."""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    node = SimpleListener()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
