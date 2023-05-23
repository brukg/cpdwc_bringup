import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
import pathlib
from launch.substitutions import EnvironmentVariable

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    package_description = "bringup"
    
   
    config_dir = os.path.join(get_package_share_directory(package_description), 'config', 'robot_localization.yaml')
    
    parameters_file_dir = pathlib.Path(__file__).resolve().parent.parent / 'config' / 'robot_localization.yaml'
    parameters_file_path =   str(parameters_file_dir)
    os.environ['FILE_PATH'] = str(parameters_file_dir)

    # rviz_node = Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         output='screen',
    #         name='rviz_node',
    #         parameters=[{'use_sim_time': False}],
    #         arguments=['-d', config_dir]
    # )

    robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            output='screen',
            name='ekf_filter_node',
            parameters=[
                parameters_file_path,
                str(parameters_file_path),
                [EnvironmentVariable(name='FILE_PATH')],
           ]
    )

    # create and return launch description object
    return LaunchDescription(
        [            
            robot_localization_node,
        ]
    )