#!/usr/bin/env python

import os
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def controller_spawning(context, *args, **kwargs):
    controllers = []

    n_robots = LaunchConfiguration('n_robots').perform(context)
    robots_file = LaunchConfiguration('robot_names_file').perform(context)
    waypoints_file = LaunchConfiguration('waypoints_file').perform(context)
    dwa_params_file = LaunchConfiguration('dwa_params_file').perform(context)

    use_sim_time = TextSubstitution(text='true')
    with open(robots_file, 'r') as stream:
        robots = yaml.safe_load(stream)
    with open(waypoints_file, 'r') as stream:
        goals = yaml.safe_load(stream)
    with open(dwa_params_file, 'r') as stream:
        dwa_params = yaml.safe_load(stream)
        
    
    for name, goal in list(zip(robots, goals))[:int(n_robots)]:
        controllers.append(Node(
           package='planning',
           executable='planner',
           remappings=[('/tf', f'tf'), ('/tf_static', f'tf_static')],
           namespace=name,
           parameters=[{
            'use_sim_time': use_sim_time,
            'vehicle_model': 2,
            'turn_radius': 0.15,
            'turn_speed': 1.0,
            'step_size': 1.0,
            'map_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps', 'lndw2022.yaml'),
            }],
           output='screen',
        ))
        controllers.append(Node(
           package='trajectory_follower',
           executable='dwa',
           namespace=name,
           parameters=[
              {
                  "use_sim_time": use_sim_time,
              }, dwa_params,
           ],
           remappings=[('/tf',"tf"), ('/tf_static',"tf_static")],
           output='screen',
        ))
        controllers.append(Node(
           package='goal_provider',
           executable='simple_goal',
           namespace=name,
           remappings=[('/tf', f'tf'), ('/tf_static', f'tf_static')],
           parameters=[{
              'use_sim_time': use_sim_time,
              'goal_radius': 0.3,
              'waypoints': yaml.dump(goal['waypoints']),
           }],
           output='screen',
           #arguments=[],
        ))
    
    return controllers


def generate_launch_description():
    args = {
         'behaviour': 'false',
         'world': 'lndw2022.world',
         'map': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'maps' ,'lndw2022.yaml'),
         'waypoints_file': os.path.join(get_package_share_directory('planning'),'params', 'robot_waypoints.yaml'),
         'poses_file': os.path.join(get_package_share_directory('planning'), 'params', 'robot_poses.yaml'),
         'robot_names_file': os.path.join(get_package_share_directory('driving_swarm_bringup'), 'params', 'robot_names_sim.yaml'),
         'dwa_params_file': os.path.join(get_package_share_directory('planning'), 'params', 'dwa_params.yaml'),
    }
    multi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('driving_swarm_bringup'), 'launch', 'multi_robot.launch.py')),
        launch_arguments=args.items())


    ld = LaunchDescription()
    ld.add_action(multi_robot_launch)
    ld.add_action(OpaqueFunction(function=controller_spawning))
    return ld