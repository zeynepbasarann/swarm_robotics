from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
import random

class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.pub1 = self.create_publisher(Point, '/set_target_position1', 10)
        self.pub2 = self.create_publisher(Point, '/set_target_position2', 10)
        self.timer = self.create_timer(10.0, self.publish_random_targets)

    def publish_random_targets(self):
        msg1 = Point()
        msg1.x = random.uniform(-1.5, 1.5)
        msg1.y = random.uniform(-1.5, 1.5)
        msg1.z = 0.1
        self.pub1.publish(msg1)

        msg2 = Point()
        msg2.x = random.uniform(-1.5, 1.5)
        msg2.y = random.uniform(-1.5, 1.5)
        msg2.z = 0.1
        self.pub2.publish(msg2)

def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
