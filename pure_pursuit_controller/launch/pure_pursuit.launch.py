from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Pure Pursuit controller
        Node(
            package='pure_pursuit_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen'
        ),
        
        # Route publisher (sin dependencia de goal_reached)
        Node(
            package='pure_pursuit_controller',
            executable='route_publisher',
            name='route_publisher',
            output='screen',
            parameters=[{
                'loop_route': False
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])