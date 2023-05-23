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

    use_sim_time = LaunchConfiguration('use_sim_time',  default='false')
    this_directory = get_package_share_directory("stage_ros2")

    launch_dir = os.path.join(this_directory, 'launch')
    stage = LaunchConfiguration('stage')
    rviz = LaunchConfiguration('rviz')
    config = LaunchConfiguration('config')
    map_yaml_file = LaunchConfiguration('map')
    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_stage_cmd = DeclareLaunchArgument(
        'stage',
        default_value='True',
        description='Whether run a stage')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Whether run a rviz')

    declare_config = DeclareLaunchArgument(
        'config', default_value='multi_robot_stage',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_world = DeclareLaunchArgument(
        'world', default_value='empty',
        description='world to load in stage')
    declare_map = DeclareLaunchArgument(
        'map', default_value=os.path.join(this_directory, 'world/bitmaps', "empty.yaml"),
        description='map to load in stage')
    
    st_map2odom0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_0/odom']
    )
    st_map2world0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_0/world']
    )
    st_map2odom1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_1/odom']
    )

    st_map2world1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_1/world']
    )
    st_map2odom2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_2/odom']
    )
    
    st_map2world2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_2/world']
    )
    st_map2odom3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_3/odom']
    )
    
    st_map2world3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_3/world']
    )
    st_map2odom4 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_4/odom']
    )

    st_map2world4 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_4/world']
    )
    st_map2odom5 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_5/odom']
    )
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[param_substitutions],

    )

    map_server_life_cycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_server_lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    controller_node = Node(
        package='pdwa',
        executable='controller',
        emulate_tty=True,
        output="screen",
        remappings=[
            ('cmd_vel', '/robot_0/cmd_vel'),
            # ('global_pose', '/robot_0/global_pose'),
            # ('cmd_vel', 'itav_agv/cmd_vel'),
            ('odom', 'robot_0/odom'),
        ],
        parameters=[
            {'rate': 10},
            {'footprint': "[[0.44, 0.38], [0.44, -0.38], [-0.44, -0.38], [-0.44, 0.38]]"},
            {'max_speed': 1.0},
            {'min_speed': -1.0},
            {'max_yawrate': 0.707},
            {'max_accel': 6.0},
            # {'max_dyawrate': 600.0},
            # {'max_dyawrate': 100.0},
            {'max_dyawrate': 300.0},
            {'v_reso': 0.03},
            {'yawrate_reso': 0.01},
            {'dt': 0.1},
            {'predict_time': 3.0},
            {'to_goal_cost_gain': 2.0},
            {'speed_cost_gain': 0.750},
            {'obstacle_cost_gain': 2.50},
            {'goal_tolerance': 0.250},
            {'reverse_penality': 15.0},
            {'robot_radius': 0.5},
            {'obstacle_radius': 0.5},
            {'obstacle_margin': 0.250},
            {'collision_threshold': 0.005},
        ],
        
    )

    transform_node = Node(
        package='pdwa',
        executable='transform.py',
        name='transform_node',
        emulate_tty=True,
        output="screen",
        remappings=[
            ('odom', 'robot_0/odom'),
            # ('odom', 'itav_agv/odom'),
            # ('global_pose', 'itav_agv/global_pose'),
        ]
    
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_rviz_cmd,
        declare_stage_cmd,
        declare_config,
        declare_world,
        declare_map,
        # map_server,
        # map_server_life_cycle_manager,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            condition=IfCondition(rviz),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'config': config}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'stage.launch.py')),
            condition=IfCondition(stage),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'map_server_launch.py')),),
        st_map2odom0,
        st_map2world0,
        st_map2odom1,
        st_map2world1,
        st_map2odom2,
        st_map2world2,
        st_map2odom3,
        st_map2world3,
        st_map2odom4,
        st_map2world4,
        st_map2odom5,
        # transform_node,
        # controller_node,
    ])
