import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleTalker(Node):
    """A small node that publishes robot status messages."""

    def __init__(self):
        super().__init__('simple_talker')

        # Create a publisher that sends String messages on /robot_status.
        self.publisher_ = self.create_publisher(String, '/robot_status', 10)

        # Run publish_status once every 1.0 second.
        self.timer = self.create_timer(1.0, self.publish_status)
        self.message_count = 0

    def publish_status(self):
        """Build and publish one status message."""
        msg = String()
        msg.data = f'Robot is running. Message #{self.message_count}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.message_count += 1


def main(args=None):
    rclpy.init(args=args)

    node = SimpleTalker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
