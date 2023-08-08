import rclpy
import numpy as np
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped


class LocatorNode(Node):
    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')
        self.x = 0
        self.y = 0
        self.z = 0
        
    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)
    
    def calculate_position(self):
        if not len(self.anchor_ranges):
            return 0.0, 0.0, 0.0
        
        adist_x =  np.array([self.x-r.anchor.x for r in self.anchor_ranges])
        adist_y =  np.array([self.y-r.anchor.y for r in self.anchor_ranges])
        adist_z =  np.array([self.z-r.anchor.z for r in self.anchor_ranges])
        
        mi= np.array([r.range for r in self.anchor_ranges])
        dist_vector = np.array([np.array([adist_x[i],adist_y[i],adist_z[i]]) for i in range(len(adist_x))])
        R_hat = np.array([mi[j] - np.linalg.norm(dist_vector[j]) for j in range(len(dist_vector))])
        R_hat_grad = np.array([-1*(dist_vector[j]/np.linalg.norm(dist_vector[j])) for j in range(len(dist_vector))])
        inverse = np.linalg.pinv(R_hat_grad)
        correction = inverse @ R_hat
        
        self.x -= correction[0]
        self.y -= correction[1]
        self.z -= correction[2]
        
        return self.x, self.y, self.z


def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
