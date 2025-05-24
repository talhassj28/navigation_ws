import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class DisplayLaser(Node):
    def __init__(self):
        super().__init__("display_laser")

        self.laser_subscriber = self.create_subscription(
            msg_type=LaserScan,
            topic="/laser/scan",
            callback=self.laser_callback,
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

    def laser_callback(self, msg: LaserScan):
        self.get_logger().info(f"Range Min: {min(msg.ranges)}")


def main(args=None):
    rclpy.init(args=args)

    display_laser = DisplayLaser()

    rclpy.spin(display_laser)

    display_laser.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
