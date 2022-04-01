import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')
        self.publisher = self.create_publisher(String, 'controller_cmd', 10)
        self.get_logger().info('Controller node started')
        self.i = 0
        self.period = 2.0
        self.create_timer(self.period, self.timer_callback)

    def timer_callback(self):
        # self.get_logger().info(f'controller @ {self.period*self.i}s')
        if self.i == 2:
            msg = String(data="start")
            self.publisher.publish(msg)
            self.get_logger().info('controller: start')
        elif self.i * self.period >= 60:
            msg = String(data="stop")
            self.publisher.publish(msg)
            self.get_logger().info('controller: stop')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
