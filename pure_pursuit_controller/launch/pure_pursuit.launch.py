from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Pure Pursuit controller
        Node(
            package='pure_pursuit_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                'lookahead_distance': 1.0,
                'max_linear_vel': 0.5,
                'max_angular_vel': 0.5,
                'goal_tolerance': 0.1
            }]
        ),
        
        # Route publisher (ahora depende de goal_reached)
        Node(
            package='pure_pursuit_controller',
            executable='route_publisher',
            name='route_publisher',
            output='screen',
            parameters=[{
                'loop_route': False
            }]
        ),
        
        # Static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])