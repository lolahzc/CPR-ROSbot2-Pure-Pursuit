from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. SIMPLE MAPPER
        Node(
            package='pure_pursuit_controller', 
            executable='simple_mapper_node',
            name='simple_mapper',
            parameters=[{'use_sim_time': True}]
        ),

        # 2. GRID PLANNER 
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

        # 3. PURE PURSUIT 
        Node(
            package='pure_pursuit_controller',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[{
                'lookahead_distance': 0.3,
                'max_linear_vel': 1.0,
                'max_angular_vel': 2.5,
                'goal_tolerance': 0.25,
                'lookahead_min': 0.1,
                'lookahead_max': 1.5,
                'lookahead_gamma': 4.0,
                'cte_threshold': 0.15
            }]
        ),
        
        # Transformación estática necesaria
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])