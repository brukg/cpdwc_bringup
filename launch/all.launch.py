import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    description_dir = get_package_share_directory('itav_agv_description')
    sick_safetyscanners2 = get_package_share_directory('sick_safetyscanners2')
    bringup_dir = get_package_share_directory('bringup')

    # cartographer_map_file = os.path.join(get_package_share_directory('bringup'), 'maps/005', 'map.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_dir, '/launch/urdf.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_dir, '/launch/agv_state.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([bringup_dir, '/launch/localization_launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([sick_safetyscanners2, '/launch/safety_all.launch.py']),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([description_dir, '/launch/controller_launch.py']),
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([description_dir, '/launch/tracker.py']),
        # ),


        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([bringup_dir, '/launch/cartographer.launch.py']),
        # ),
    ])