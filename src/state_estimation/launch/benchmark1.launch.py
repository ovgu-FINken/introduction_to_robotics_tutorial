#!/usr/bin/env python

import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def controller_spawning(context, *args, **kwargs):
    controllers = []

    n_robots = LaunchConfiguration('n_robots').perform(context)
    robots_file = LaunchConfiguration('robot_names_file').perform(context)
    goals_file = LaunchConfiguration('waypoints_file').perform(context)
    use_sim_time = TextSubstitution(text='true')
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
    with open(goals_file, 'r') as stream:
        goals = yaml.safe_load(stream)
        
    for goal, robot in list(zip(goals, robots))[:int(n_robots)]:
        controllers.append(Node(
            package='fake_range',
            executable='fake_range',
            namespace=robot,
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            parameters=[{
                'use_sim_time': use_sim_time,
                'rate' : 0.1,
                'anchor_list': '''
- {x: 0.0, y: -0.0, z: 1.0, mu: 1.05, sigma: 0.05}
- {x: 3.0, y: -3.0, z: 1.0, mu: 1.05, sigma: 0.05}
- {x: 0.0, y: -3.0, z: 1.0, mu: 1.05, sigma: 0.05}
- {x: 3.0, y: -0.0, z: 1.0, mu: 1.05, sigma: 0.05}
- {x: 0.0, y: -0.0, z: 2.0, mu: 1.05, sigma: 0.05}
- {x: 3.0, y: -3.0, z: 2.0, mu: 1.05, sigma: 0.05}
- {x: 0.0, y: -3.0, z: 2.0, mu: 1.05, sigma: 0.05}
- {x: 3.0, y: -0.0, z: 2.0, mu: 1.05, sigma: 0.05}
''' 
            }],
            output='screen',
        ))
        controllers.append(Node(
            package='state_estimation',
            executable='locator',
            namespace=robot,
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ))
        controllers.append(Node(
            package='state_estimation',
            executable='controller',
            namespace=robot,
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ))
        controllers.append(Node(
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            package='state_estimation',
            executable='scoring',
            namespace=robot,
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ))
        controllers.append(Node(
           remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
           package='goal_provider',
           executable='simple_goal',
           namespace=robot,
           parameters=[{
              'use_sim_time': use_sim_time,
              'waypoints': yaml.dump(goal['waypoints']),
           }],
           output='screen',
           #arguments=[],
        ))
    return controllers


def generate_launch_description():
    args = {
         'behaviour': 'false',
         'world': 'icra2021_no_obstacle.world',
         'map': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps' ,'icra2021_map_no_obstacle.yaml'),
         #'robots_file': os.path.join(get_package_share_directory('state_estimation'), 'params', 'robot.yaml'),
         'poses_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'icra2021_poses.yaml'),
         'robot_names_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'robot_names_sim.yaml'),
         'waypoints_file': os.path.join(get_package_share_directory('state_estimation'), 'params', 'icra_2021_real_waypoints.yaml'),
         
    }
    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('driving_swarm_bringup'), 'launch', 'multi_robot.launch.py')),
        launch_arguments=args.items())

    ld = LaunchDescription()
    ld.add_action(multi_robot_launch)
    ld.add_action(OpaqueFunction(function=controller_spawning))
    return ld
