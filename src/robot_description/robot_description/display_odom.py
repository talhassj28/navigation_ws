import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class DisplayOdom(Node):
    def __init__(self):
        super().__init__("display_odom")

        self.odom_subscriber = self.create_subscription(
            msg_type=Odometry,
            topic="/odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        self.cmd_publisher = self.create_publisher(
            msg_type=Twist, topic="/cmd_vel", qos_profile=10
        )

        # Initialize twist message
        self.twist_msg = Twist()

        # Set twist message values
        self.twist_msg.linear.x = 0.5
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.75

        # Publish twist message
        self.cmd_publisher.publish(self.twist_msg)

    def odom_callback(self, msg):
        self.get_logger().info(
            f"PoseX: {msg.pose.pose.position.x}, PoseY: {msg.pose.pose.position.y}"
        )


def main(args=None):
    rclpy.init(args=args)

    display_odom = DisplayOdom()

    rclpy.spin(display_odom)

    display_odom.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
