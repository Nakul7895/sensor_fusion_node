import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        self.create_subscription(LaserScan, '/lidar_data', self.lidar_callback, 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_brake', 10)

    def lidar_callback(self, msg):
        if msg.ranges:
            min_distance = min(msg.ranges)
            self.get_logger().info(f'Min distance: {min_distance:.2f}')
            if min_distance < 1.0:
                self.get_logger().warn('Too close! Triggering emergency brake.')
                self.emergency_pub.publish(Bool(data=True))
            else:
                self.emergency_pub.publish(Bool(data=False))

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
