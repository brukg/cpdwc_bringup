import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    package_description = "cpdwc_bringup"

    print("launching rviz")
    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'config', 'agv_vis.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': False}],
            arguments=['-d', rviz_config_dir]
    )

    # create and return launch description object
    return LaunchDescription(
        [            
            rviz_node,
        ]
    )