import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WatchdogNode(Node):

    def __init__(self):
        super().__init__('watchdog')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'input_cmd', self.cmd_callback, 10)
        self.create_subscription(String, 'controller_cmd', self.controller_callback, 10)
        self.get_logger().info('Watchdog node started')

    def cmd_callback(self, msg):
        # this makes the turle go backwards
        # (just so you know its working)
        msg.linear.x = -1 * msg.linear.x
        self.get_logger().info(f'msg.linear.x: {msg.linear.x}')
        self.publisher.publish(msg)
        
    def controller_callback(self, msg):
        self.get_logger().warn(f'The controller says I should {msg.data} the turtle ...')



def main(args=None):
    rclpy.init(args=args)

    node = WatchdogNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
