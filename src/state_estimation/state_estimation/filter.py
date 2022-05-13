import rclpy
from rclpy.node import Node


class FilterNode(Node):
    def __init__(self):
        super().__init__('filtering_node')
        self.get_logger().info('filtering node started')
        

def main(args=None):
    rclpy.init(args=args)

    node = FilterNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
