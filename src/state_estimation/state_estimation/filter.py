import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
import PyKDL

def quaternion_from_yaw(yaw):
    return PyKDL.Rotation.RotZ(yaw).GetQuaternion()


class FilterNode(Node):
    def __init__(self):
        super().__init__('filtering_node')
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.get_logger().info('filtering node started')
        
    def position_cb(self, msg):
        pose = self.update_pose(msg.point)
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = pose[0]
        pose_msg.pose.position.y = pose[1]
        pose_msg.pose.position.z = 0.0
        x, y, z, w = quaternion_from_yaw(pose[2])
        pose_msg.pose.orientation.x = x
        pose_msg.pose.orientation.y = y
        pose_msg.pose.orientation.z = z
        pose_msg.pose.orientation.w = w
        self.pose_pub.publish(pose_msg)

    def update_pose(self, position):
        # YOUR CODE GOES HERE
        return position.x, position.y, 0.0


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
