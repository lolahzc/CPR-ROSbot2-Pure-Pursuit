from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    selected_route = LaunchConfiguration('selected_route', default='1')
    return LaunchDescription([
        Node(
            package='pure_pursuit_controller', 
            executable='simple_mapper_node',
            name='simple_mapper',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='pure_pursuit_controller',
            executable='grid_planner_node',
            name='grid_planner',
            output='screen',
            parameters=[{
                'interpolation_points_per_segment': 100, 
                'knot_distance': 2.0 
            }]
        ),

        Node(
            package='pure_pursuit_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                'lookahead_distance': 0.3,

                'max_linear_vel': 0.5,
                'max_angular_vel': 1.5,
                'goal_tolerance': 0.25,

                'lookahead_min': 0.3,
                'lookahead_max': 1.0,
                'lookahead_gamma': 25.0,

                'bubble_base_radius': 1.0,
                'critical_distance': 0.25,
                'detour_offset': 0.75,
                'rejoin_distance': 1.5,
                'forward_offset': 1.5,

                'selected_route': selected_route
            }]
        ),

        Node(
            package='pure_pursuit_controller',
            executable='route_publisher',
            name='route_publisher',
            output='screen',
            parameters=[{
                'loop_route': False,
                'interpolation_points_per_segment': 50,
                'selected_route': selected_route
            }]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])