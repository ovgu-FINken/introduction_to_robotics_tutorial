import rclpy
from rclpy.node import Node

from driving_swarm_nav_graph.nav_graph import NavGraphNode
from trajectory_generator.vehicle_model_node import (
    TrajectoryGenerator,
    Vehicle,
)

import tf2_ros
import tf2_kdl
import tf2_py
import tf2_geometry_msgs
import numpy as np
import traceback
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from std_msgs.msg import String, Int32
from driving_swarm_messages.srv import UpdateTrajectory
from std_srvs.srv import Empty
from trajectory_generator.utils import yaw_from_orientation, yaw_to_orientation
import polygonal_roadmaps as poro
import yaml
from planning import utils


class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.get_logger().info("Starting")

        # initialize local variables
        self.own_frame = "base_link"
        self.reference_frame = "map"
        self.goal = None
        self.pose = None

        self.started = False
        self.current_trajectory = None

        # declare parameters for vehicle model
        self.declare_parameter("vehicle_model")
        self.declare_parameter("step_size")
        self.declare_parameter("turn_radius")
        self.declare_parameter("turn_speed")
        self.declare_parameter("map_file")
        
        # create vehicle model
        self.vm = TrajectoryGenerator(
            model=Vehicle(
                self.get_parameter("vehicle_model")
                .get_parameter_value()
                .integer_value
            ),
            step=0.1,
            r=self.get_parameter("turn_radius")
            .get_parameter_value()
            .double_value,
            r_step=1.0,
        )

        self.occupied_space = utils.calculate_occupied_space(self.get_parameter('map_file').get_parameter_value().string_value)
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        qos_profile = rclpy.qos.qos_profile_system_default
        qos_profile.reliability = (
            rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        )
        qos_profile.durability = (
            rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )
        self.status_pub = self.create_publisher(String, "status", qos_profile)

        self.create_subscription(
            String, "/command", self.command_cb, qos_profile
        )
        self.follow_client = self.create_client(
            UpdateTrajectory, "nav/follow_trajectory"
        )

        # create replanning service
        self.create_service(Empty, "nav/replan", self.replan_callback)
        self.follow_client.wait_for_service()
        self.get_logger().info("connected to trajectory follower service")
        f = self.tfBuffer.wait_for_transform_async(
            self.own_frame, self.reference_frame, rclpy.time.Time().to_msg()
        )
        self.get_logger().info("waiting for transform map -> baselink")
        rclpy.spin_until_future_complete(self, f)
        self.timer_cb()
        self.create_subscription(PoseStamped, "nav/goal", self.goal_cb, 1)
        self.status_pub.publish(String(data="ready"))
        
        # start timer behaviour
        self.create_timer(1.0, self.timer_cb)
    
    
    def timer_cb(self):
        try:
            trans = self.tfBuffer.lookup_transform(
                self.reference_frame,
                self.own_frame,
                rclpy.time.Time().to_msg(),
            )
            frame = tf2_kdl.transform_to_kdl(trans)
            pose = (frame.p.x(), frame.p.y(), frame.M.GetRPY()[2])

        except Exception as e:
            self.get_logger().warn(f"Exception in tf transformations\n{e}")
            return

        # current pose (x, y, angle) is stored in self.pose     
        self.pose = pose
        

    def goal_cb(self, msg):
        yaw = yaw_from_orientation(msg.pose.orientation)
        goal = msg.pose.position.x, msg.pose.position.y, yaw
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]}, angle={goal[2]})')
            self.goal = goal
            self.go_to_goal()            

            
    def command_cb(self, msg):
        if msg.data == "go":
            self.get_logger().info("going")
            self.started = True
            self.go_to_goal()
    
    def go_to_goal(self):
        path = self.create_plan()
        if path is None:
            return
        self.send_path(path)

    # send path will send the path to the trajectory follower
    def send_path(self, path):
        # check if path is colliding:
        if self.occupied_space.intersects(utils.path_2d(path)):
            self.get_logger().warn("path will collide with obstacle")

        # convert trajectory to correct space
        if path is None:
            return
        path_msg = Path()
        path_msg.header.frame_id = self.reference_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for pose in path:
            pose3d = PoseStamped()
            pose3d.header.frame_id = self.reference_frame
            pose3d.header.stamp = self.get_clock().now().to_msg()
            pose3d.pose.position.x = pose[0]
            pose3d.pose.position.y = pose[1]
            pose3d.pose.position.z = 0.0
            pose3d.pose.orientation = yaw_to_orientation(pose[2])
            path_msg.poses.append(pose3d)

        self.get_logger().info("sending path")
        path_msg.header.stamp = self.get_clock().now().to_msg()
        request = UpdateTrajectory.Request(trajectory=path_msg, update_index=0)
        self.follow_client.call_async(request)

    def create_plan(self):
        if self.goal is None:
            self.get_logger().info("ignoring path without goal")
            return
        if self.pose is None:
            self.get_logger().info("ignoring path without current pose")
            return
        path = self.vm.tuples_to_path([self.pose, self.goal])
        
        return path
    
    def replan_callback(self, _, res):
        if self.started:
            self.get_logger().info('got replanning request, which we will do')
        self.go_to_goal()
        return res
    

def main():
    rclpy.init()
    node = Planner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
