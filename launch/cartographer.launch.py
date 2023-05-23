import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    itav_agv_cartographer_prefix = get_package_share_directory('bringup')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  itav_agv_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer_2d.lua')
    
    cartographer_map_file = os.path.join(get_package_share_directory('bringup'), 'maps/one', 'b.pbstream')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.5')
    finish = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/finish_trajectory', 'cartographer_ros_msgs/srv/FinishTrajectory',
                    '{trajectory_id: 0}'],
            output='screen'
        )
    start = ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/start_trajectory', 'cartographer_ros_msgs/srv/StartTrajectory', 
                 '{configuration_directory: "/home/phoenix/ros2_ws/src/tavil/bringup/config", configuration_basename:  "cartographer_2d.lua", use_initial_pose: false, relative_to_trajectory_id: 1, initial_pose: {position: {x: 20., y: 20., z: 0.}, orientation: {x: 0., y: 0., z: 0., w: 1.}}}'],
            output='screen'
        ),
    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true'),
        # DeclareLaunchArgument(
        #     'map',
        #     default_value=map,
        #     description='Full path to map file to load'),


        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            remappings=[
                ('odom', 'itav_agv/odom'),
                # ('odom', 'odometry/filtered'),
                ('scan_1', 'itav_agv/safety_lidar_front_link/scan'),
                # ('scan_1', 'safety_lidar_front_link/scan'),
                ('scan_2', 'itav_agv/safety_lidar_back_link/scan'),
                # ('scan_2', 'safety_lidar_back_link/scan'),
                # ('points2_1', 'itav_agv/lidar_main/scan'),
                # ('points2_2', 'itav_agv/safety_lidar_front/scan'),
                # ('points2_3', 'itav_agv/safety_lidar_back/scan')
            ],
            parameters=[{'use_sim_time': use_sim_time}, 
                        ],
            arguments=[
                        '-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '-initial_pose "{initial_pose = { position = { 20., 20., 0. }, rotation = { 0., 0., 0., 1. } } }"',
                       '-relative_to_trajectory_id = 0,'
                        # '-load_state_filename', cartographer_map_file,
                        # '-load_frozen_state', 'true',                       
                       ]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
        #                       'publish_period_sec': publish_period_sec}.items(),
        # ),
        
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=finish,
        #         on_exit=[ExecuteProcess(start)],
        #     ),
        # ),
        

    ])