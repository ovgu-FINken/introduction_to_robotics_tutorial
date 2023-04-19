#!/usr/bin/env python

import numpy as np
from skimage import io, segmentation
from scipy.ndimage import gaussian_filter

import rclpy
import tf2_ros
import tf2_geometry_msgs
import tf2_py
import tf_transformations
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from driving_swarm_utils.node import DrivingSwarmNode

def yaw_from_orientation(orientation):
    ot = orientation.x, orientation.y, orientation.z, orientation.w
    return tf_transformations.euler_from_quaternion(ot)[2]

class ScoringNode(DrivingSwarmNode):
    def __init__(self):
        super().__init__('scoring_node')

        self.robot_frames = []
        self.get_logger().info(self.get_namespace())
        self.get_frames()
        self.setup_tf()
        
        self.safety_dist = 0.15

        
        self.raw_map = None
        # create a qos profile to receive the last sent map
        qos_profile = rclpy.qos.qos_profile_system_default
        qos_profile.reliability = rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        qos_profile.durability = rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, 'map', self.map_cb, qos_profile)
        self.map_publisher = self.create_publisher(OccupancyGrid, 'score_grid', 10)
        self.scan = None
        self.map_client = self.create_client(GetMap, 'map_server/map')
        self.map_client.wait_for_service()
        future = self.map_client.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(self, future)
        self.wait_for_tf()
        self.score_image = None
        self.update_map(future.result().map)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, rclpy.qos.qos_profile_sensor_data)
        self.t = 0
        self.create_timer(0.5, self.timer_cb)
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.status_pub.publish(String(data='ready'))
        self.get_logger().info('init done')
        
    def scan_cb(self, msg):
        self.scan = msg
        
    def position_image(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.raw_map.header.frame_id, self.own_frame, rclpy.time.Time().to_msg())
        except Exception as e:
            self.get_logger().warn(f'{e}')
            return np.zeros_like(self.map_image)
        
        position_image = np.zeros_like(self.map_image)
        x, y = self.transform_to_px_coordinates(trans)
        position_image[y-2:y+3, x-2:x+3] = 100
        
        return position_image
        
    def convert_map_data_to_image(self, map_data):
        image = np.array(map_data.data, dtype=int)
        image = np.reshape(image, (map_data.info.height, map_data.info.width))
        image[image < 50] = 0
        image[image >= 50] = 100
        return image
    
    def metric_to_px_coorditaes(self, x, y):
        x = (x - self.raw_map.info.origin.position.y ) / self.raw_map.info.resolution
        y = (y - self.raw_map.info.origin.position.x ) / self.raw_map.info.resolution
        return int(x), int(y)

    def transform_to_px_coordinates(self, t):
        x = (t.transform.translation.x - self.raw_map.info.origin.position.x) / self.raw_map.info.resolution
        y = (t.transform.translation.y - self.raw_map.info.origin.position.y) / self.raw_map.info.resolution
        return int(x), int(y)


    def combine_sensor_sources(self):
        img = np.maximum(self.position_image(), self.map_image)
        if self.score_image is None:
            self.score_image = img
        else:
            self.score_image = np.maximum(img, self.score_image)
        return self.score_image
    
    def convert_image_to_map_data(self, image):
        map_data = self.raw_map
        map_data.data = [int(x) for x in image.flat]
        return map_data
    
    def update_map(self, map_msg):
        self.raw_map = map_msg
        #extract map as image
        self.map_image = self.convert_map_data_to_image(map_msg)
    
    def map_cb(self, msg):
       self.get_logger().info('received a map callback\n \n \n xxxxxxxxxxxxxxxxxxxxxxxxxx \n \n \n') 
       self.update_map(msg)
       
    def timer_cb(self):
        if self.raw_map is not None:
            self.map_publisher.publish(self.convert_image_to_map_data(self.combine_sensor_sources()))
        else:
            self.get_logger().info('no map received yet')
        if self.score_image is not None:
            self.get_logger().info(f'score: {np.sum(self.score_image)/10_000} at t={0.5*self.t}s')
        self.t+=1
   
def main():
    rclpy.init()
    node = ScoringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting Down')
        node.destroy_node()

if __name__ == "__main__":
    main()
