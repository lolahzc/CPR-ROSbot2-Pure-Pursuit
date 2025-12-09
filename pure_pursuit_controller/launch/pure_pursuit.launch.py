from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declaración de configuración para selected_route
    selected_route = LaunchConfiguration('selected_route', default='4')

    return LaunchDescription([
        # Pure Pursuit controller
        Node(
            package='pure_pursuit_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                'lookahead_distance': 0.5,
                'max_linear_vel': 0.5,
                'max_angular_vel': 0.5,
                'goal_tolerance': 0.25
            }]
        ),

        # Route publisher con interpolación
        Node(
            package='pure_pursuit_controller',
            executable='route_publisher',
            name='route_publisher',
            output='screen',
            parameters=[{
                'loop_route': False,
                'interpolation_points_per_segment': 10,
                'selected_route': selected_route
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
