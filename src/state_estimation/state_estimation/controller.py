import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
import tf_transformations
import math
import copy
class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.goal = None
        self.position = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose_marker', 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        self.angle = 0
        self.target_angle = None
        self.points = 0
    def timer_cb(self):
        msg = Twist()
        if self.position == None or self.goal == None:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            self.target_angle = np.rad2deg(np.arctan((self.goal[1]-self.position[1])/(self.goal[0]-self.position[0])))
            if self.goal[0]-self.position[0] < 0:
                self.target_angle -= 180
            #if self.position[1]-self.goal[1] > 0 and self.position[0]-self.goal[0] > 0:
            #    self.target_angle -= 180
            if abs(self.target_angle-self.angle) > 10:
                if self.angle-self.target_angle < 0:
                    msg.angular.z = 0.4
                elif self.angle-self.target_angle > 0:
                    msg.angular.z = -0.4
                msg.linear.x = 0.01
            else:
                msg.linear.x = 0.1
                msg.angular.z = 0.0
            
            #self.get_logger().info(str(self.angle)+"         "+str(self.target_angle))
            #self.get_logger().info(str(self.goal[1]-self.position[1])+"        "+str(self.goal[0]-self.position[0]))
            #self.get_logger().info(str(math.sqrt((self.goal[1]-self.position[1])**2+(self.goal[0]-self.position[0])**2)))

        if self.forward_distance < 0.3:
            msg.linear.x = 0.001
            msg.angular.z = -0.1
        #elif self.right_distance < 0.3:
        #    msg.angular.z = 0.1
        #elif self.left_distance < 0.3:
        #    msg.angular.z = -0.1

        self.publisher.publish(msg)
        angle = 0
            
        self.angle += 6*msg.angular.z
        self.publish_marker((0,0), angle, relative=True)

        #if self.angle > 90:
        #    self.angle -= 180
        #elif self.angle < -90:
        #    self.angle += 180
    
    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y
        
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
            #self.angle = 0
    
    def laser_cb(self, msg):
        self.forward_distance = msg.ranges[0]
        self.left_distance = msg.ranges[90]
        self.right_distance = msg.ranges[270]
    def position_cb(self, msg):
        self.position = msg.point.x, msg.point.y
    
    def publish_marker(self, position, angle, relative=False):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if not relative:
            msg.header.frame_id = 'map'
        else:
            msg.header.frame_id = 'base_link'

        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, angle)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.pose_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

