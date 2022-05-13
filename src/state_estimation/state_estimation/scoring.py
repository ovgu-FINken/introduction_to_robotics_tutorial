import rclpy
from rclpy.node import Node


class ScoringNode(Node):

    def __init__(self):
        super().__init__('scoring_node')
        self.get_logger().info('scoring node started')
        


def main(args=None):
    rclpy.init(args=args)

    node = ScoringNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
