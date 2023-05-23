import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():
    ld = LaunchDescription()
    param_file = os.path.join(get_package_share_directory('pdwa'), 'params', 'params.yaml')
    namespace = os.environ['AGV_NAME']

    controller_node = Node(
        package='pdwa',
        executable='controller',
        emulate_tty=True,
        output="screen",
        namespace=namespace,
        remappings=[
            ('goal_pose', '/goal_pose'),
        ],
        parameters=[
            {'rate': 10},
            {'footprint': "[[1.7, 0.6], [1.7, -0.6], [-0.6, -0.6], [-0.6, 0.6]]"},
            {'max_speed': 1.65},
            {'min_speed': -1.65},
            {'max_yawrate': 0.707},
            {'max_accel': 6.0},
            # {'max_dyawrate': 600.0},
            # {'max_dyawrate': 100.0},
            {'max_dyawrate': 300.0},
            {'v_reso': 0.3},
            {'yawrate_reso': 0.1},
            {'dt': 0.1},
            {'predict_time': 3.0},
            {'to_goal_cost_gain': 1.0},
            {'speed_cost_gain': 0.750},
            {'obstacle_cost_gain': 3.50},
            {'goal_tolerance': 0.250},
            {'reverse_penality': 15.0},
            {'robot_radius': 0.5},
            {'obstacle_radius': 0.75},
            {'obstacle_margin': 1.250},
            {'collision_threshold': 50.0},
        ],
        
    )
    transform_node = Node(
        package='pdwa',
        executable='transform.py',
        name='transform_node',
        emulate_tty=True,
        namespace=namespace,
        output="screen",
    
    )
    ld.add_action(controller_node)
    ld.add_action(transform_node)
    # create and return launch description object
    return ld