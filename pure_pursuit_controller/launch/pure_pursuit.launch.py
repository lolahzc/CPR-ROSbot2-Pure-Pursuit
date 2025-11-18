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
                'lookahead_distance': 2.0,
                'max_linear_vel': 0.75,
                'max_angular_vel': 1.0,
                'goal_tolerance': 0.1
            }]
        ),
        
        # Route publisher (sin dependencia de goal_reached)
        Node(
            package='pure_pursuit_controller',
            executable='route_publisher',
            name='route_publisher',
            output='screen',
            parameters=[{
                'waypoint_duration': 4.0, 
                'loop_route': False
            }]
        ),
    ])