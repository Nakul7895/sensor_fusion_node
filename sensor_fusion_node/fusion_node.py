import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import LaserScan


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Initialize sensor values
        self.lidar_distance = float('inf')
        self.radar_distance = float('inf')
        self.camera_detected = False

        # Subscribe to LIDAR, radar, and camera
        self.create_subscription(LaserScan, '/lidar_data', self.lidar_callback, 10)
        self.create_subscription(Float32, '/radar_distance', self.radar_callback, 10)
        self.create_subscription(Bool, '/camera_alert', self.camera_callback, 10)

        # Publisher for emergency braking signal
        self.emergency_pub = self.create_publisher(Bool, '/emergency_brake', 10)

        self.get_logger().info("Sensor Fusion Node started...")

    def lidar_callback(self, msg):
        if msg.ranges:
            self.lidar_distance = min(msg.ranges)
        self.evaluate_emergency()

    def radar_callback(self, msg):
        self.radar_distance = msg.data
        self.evaluate_emergency()

    def camera_callback(self, msg):
        self.camera_detected = msg.data
        self.evaluate_emergency()

    def evaluate_emergency(self):
        # Fusion logic: Brake if either lidar or radar detects object & camera confirms
        emergency = False

        if (self.lidar_distance < 1.0 or self.radar_distance < 1.5) and self.camera_detected:
            emergency = True
            self.get_logger().warn("🚨 Emergency Brake Triggered!")
        else:
            self.get_logger().info("All clear. No brake.")

        self.emergency_pub.publish(Bool(data=emergency))


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
