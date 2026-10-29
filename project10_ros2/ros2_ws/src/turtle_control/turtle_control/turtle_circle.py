import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class TurtleCircle(Node):
    """Parameterized publisher with a enable/disable service."""

    def __init__(self):
        super().__init__("turtle_circle")

        self.declare_parameter("linear_speed", 2.0)
        self.declare_parameter("angular_speed", 1.0)
        self.declare_parameter("publish_rate", 10.0)

        self.enabled = True
        self.publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )
        self.service = self.create_service(
            SetBool, "~/set_enabled", self.on_set_enabled
        )

        rate = max(float(self.get_parameter("publish_rate").value), 1.0)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        if not self.enabled:
            return

        linear = float(self.get_parameter("linear_speed").value)
        angular = float(self.get_parameter("angular_speed").value)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

    def on_set_enabled(self, request, response):
        self.enabled = bool(request.data)

        if not self.enabled:
            self.publisher.publish(Twist())

        response.success = True
        response.message = (
            "circle enabled" if self.enabled else "circle stopped"
        )
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
