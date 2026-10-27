import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseListener(Node):
    """Print the turtle pose published by turtlesim."""

    def __init__(self):
        super().__init__("pose_listener")
        self.subscription = self.create_subscription(
            Pose, "/turtle1/pose", self.callback, 10
        )

    def callback(self, msg):
        self.get_logger().info(
            "x=%.2f, y=%.2f, theta=%.2f" % (msg.x, msg.y, msg.theta)
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
