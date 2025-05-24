import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


class DisplayImu(Node):
    def __init__(self):
        super().__init__("display_imu")

        self.imu_subscriber = self.create_subscription(
            msg_type=Imu,
            topic="/imu",
            callback=self.imu_callback,
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

    def imu_callback(self, msg: Imu):
        self.get_logger().info(f"LinAcc_X: {msg.linear_acceleration.x}")
        self.get_logger().info(f"AngVel_Z: {msg.angular_velocity.z}")
        self.get_logger().info(f"qX: {msg.orientation.x}, qY: {msg.orientation.y}")
        self.get_logger().info(f"qZ: {msg.orientation.z}, qW: {msg.orientation.w}")


def main(args=None):
    rclpy.init(args=args)

    display_imu = DisplayImu()

    rclpy.spin(display_imu)

    display_imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
