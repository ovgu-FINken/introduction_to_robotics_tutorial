import rclpy
import numpy as np
from driving_swarm_utils.node import DrivingSwarmNode

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32

import tf2_ros

class ScoringNode(DrivingSwarmNode):

    def __init__(self):
        super().__init__('scoring_node')
        self.error = []
        self.setup_tf()
        self.get_frames()
        self.wait_for_tf()
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.score_pub = self.create_publisher(Float32, 'score', 10)
        self.create_timer(1.0, self.timer_cb)
        
    def position_cb(self, msg):
        try:
            t = self.tf_buffer.lookup_transform(self.reference_frame, self.own_frame, rclpy.time.Time().to_msg())
        except Exception as e:
            self.get_logger().warn(f'{e}')
            return
        x = t.transform.translation.x
        y = t.transform.translation.y
        z = 0 #t.transform.translation.z
        
        dx = x - msg.point.x
        dy = y - msg.point.y
        dz = z - msg.point.z
        self.error.append(np.linalg.norm([dx, dy, dz])**2)
        self.get_logger().info(f'e = {self.error[-1]}')
        
    def timer_cb(self):
        if not len(self.error):
            return
        self.score_pub.publish(Float32(data=np.sqrt(np.mean(self.error))))

        

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
