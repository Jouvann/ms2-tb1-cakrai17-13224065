import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class HubungNode(Node):
    def __init__(self):
        super().__init__('hubung_node')

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_type_publisher = self.create_publisher(String, 'cmd_type', 10)

        self.autonomous_vel_subscriber = self.create_subscription(
            Twist,
            'autonomous_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.cmd_vel_publisher.publish(msg)

        cmd_type_msg = String()
        cmd_type_msg.data = 'autonomous'
        self.cmd_type_publisher.publish(cmd_type_msg)

def main(args=None):
    rclpy.init(args=args)

    node = HubungNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()