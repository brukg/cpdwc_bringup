#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    num_robots = 1
    controller_node_ = [None] * num_robots
    transform_node_ = [None] * num_robots
    scan2pt_ = [None] * num_robots
    tracker_ = [None] * num_robots

    for i in range(num_robots):
        robot_name = "robot_" + str(i)
        controller_node_[i] = Node(
            package='cpdwc',
            executable='controller',
            name='controller_node' + str(i),
            emulate_tty=True,
            namespace=robot_name,
            output="screen",
            remappings=[
                ('goal_pose', '/goal_pose'),
            ],
            parameters=[
                {'rate': 50},
                {'footprint': "[[0.30, 0.25], [0.30, -0.25], [-0.30, -0.25], [-0.30, 0.25]]"},
                {'max_speed': 0.750},
                {'min_speed': -0.750},
                {'max_yawrate': 0.96},
                {'max_accel': 6.0},
                # {'max_dyawrate': 600.0},
                # {'max_dyawrate': 100.0},
                {'max_dyawrate': 300.0},
                {'v_reso': 0.1},
                {'yawrate_reso': 0.6},
                {'dt': 0.1},
                {'predict_time': 3.0},
                {'to_goal_cost_gain': 1.0},
                {'speed_cost_gain': 0.750},
                {'obstacle_cost_gain': 3.50},
                {'goal_tolerance': 0.750},
                {'reverse_penality': 3.50},
                {'robot_radius': 0.5},
                {'obstacle_radius': 0.75},
                {'obstacle_margin': 1.25},
                {'collision_threshold': 5.0},
            ],
        )
        transform_node_[i] = Node(
            package='cpdwc',
            executable='transform.py',
            name='transform_node' + str(i),
            namespace=robot_name,
            emulate_tty=True,
            output="screen",
            remappings=[
                # ('odom', 'robot_0/odom'),
                # ('odom', 'itav_agv/odom'),
                # ('global_pose', 'robot_0/global_pose'),
            ]
        )
        scan2pt_[i] = Node(
            package='cpdwc_tracker',
            executable='scan2pc.py',
            name='scan2pointcloud2',
            namespace=robot_name,
            remappings=[
                ("scan", "base_scan"),
            ]
        )
            
        tracker_[i] = Node(
            package='cpdwc_tracker',
            executable='cpdwc_tracker',
            name='cpdwc_tracker' + str(i),
            namespace=robot_name,
            parameters= [
            {'visualize': True},
            {'publish_rate': 10},
            # {'publish_rate': 20},
            {'map_frame': 'map'},
            {'clustering_tolerance': 0.3},
            # {'clustering_tolerance': 0.50},
            {'min_cluster_size': 1.0},
            # {'min_cluster_size': 2.0},
            {'max_cluster_size': 5000.0},
            {'points_threshold': 500},
            {'max_distance': 2.0},
            {'max_expected_velocity': 5.0},
            {'scanner_range': 10.0},

            # {'cluster_search_method': 'dbscan'},
            {'cluster_search_method': 'kdtree'},
            # {'cluster_search_method': 'meanshift'},
            {'bandwidth': 1.0},
            {'convergence_threshold': 10.0},
            {'max_iterations': 1000},
            # {'tracker_type': 'kf'},
            {'tracker_type': 'enkf'},
            {'sliding_window_size': 12},
            {'state_size': 4},
            {'control_size': 0},
            {'measurement_size': 4},

            {'data_association_type': 'hungarian'},
            # {'data_association_type': 'greedy'},
            # {'data_association_type': 'jpda'},
            # {'data_association_type': 'jcbb'},
            {'ensemble_size': 5},
            {'predict_num_with_no_update': 20},

        ],
        remappings=[
            ("in_1", "base_scan"),
            ("in_2", "base_scan2"),
            ("out", "itav_agv/cpdwc_tracker/point_clusters"),
        ],
    )
    
    return LaunchDescription([ i for i in controller_node_ ] + 
                             [ i for i in transform_node_ ] + 
                             [ i for i in scan2pt_] + 
                             [ i for i in tracker_ ]
                             )
