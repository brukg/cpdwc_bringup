#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('cpdwc_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time',  default='false')
    this_directory = get_package_share_directory("stage_ros2")

    launch_dir = os.path.join(this_directory, 'launch')
    stage = LaunchConfiguration('stage')
    rviz = LaunchConfiguration('rviz')
    params_file = LaunchConfiguration('params_file')
    config = LaunchConfiguration('config')
    map_yaml_file = LaunchConfiguration('map')
    world = LaunchConfiguration('world')
    namespace = LaunchConfiguration('namespace')
    use_respawn = LaunchConfiguration('use_respawn')
    lifecycle_nodes = ['map_server']
    
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_stage_cmd = DeclareLaunchArgument(
        'stage',
        default_value='True',
        description='Whether run a stage')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Whether run a rviz')

    declare_config = DeclareLaunchArgument(
        'config', default_value='multi_robot_stage',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_world = DeclareLaunchArgument(
        'world', default_value='cave_multi',
        description='world to load in stage')
    declare_map = DeclareLaunchArgument(
        # 'map', default_value=os.path.join(bringup_dir, 'maps', "tavil.yaml"),
        'map', default_value=os.path.join(bringup_dir, 'maps', "cave", "cave.yaml"),
        description='map to load in stage')
    
    st_map2odom0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'robot_0/odom']
    )

    st_world2map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
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
        name='map_server',
        parameters=[configured_params],
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0

    )

    map_server_life_cycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_rviz_cmd,
        declare_stage_cmd,
        declare_params_file_cmd,
        declare_use_respawn_cmd,
        declare_config,
        declare_world,
        declare_map,
        map_server,
        map_server_life_cycle_manager,
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
        st_world2map,
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
    ])
