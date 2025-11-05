#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Port for SF45B LiDAR'
    )
    
    high_angle_limit_arg = DeclareLaunchArgument(
        'high_angle_limit',
        default_value='160',
        description='High angle limit in degrees (positive)'
    )
    
    low_angle_limit_arg = DeclareLaunchArgument(
        'low_angle_limit',
        default_value='-160',
        description='Low angle limit in degrees (negative)'
    )

    provide_odom_frame_arg = DeclareLaunchArgument(
        'provide_odom_frame',
        default_value='false',
        description='Whether to provide odom frame'
    )
    
    expected_sensor_ids_arg = DeclareLaunchArgument(
        'expected_sensor_ids',
        default_value='["scan"]',
        description='Expected sensor IDs for cartographer'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution for occupancy grid'
    )
    
    publish_period_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='Publish period for occupancy grid'
    )

    slam_package_dir = FindPackageShare('slam')
    
    config_dir = PathJoinSubstitution([slam_package_dir, 'config'])

    # SF45B driver node
    sf45b_node = Node(
        package='sf45b',
        executable='sf45b',
        name='sf45b_driver',
        parameters=[{
            'port': LaunchConfiguration('lidar_port'),
            'highAngleLimit': LaunchConfiguration('high_angle_limit'),
            'lowAngleLimit': LaunchConfiguration('low_angle_limit')
        }],
        output='screen'
    )

    # Static transform publisher node
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'laser', 'laser_frame'],
        output='screen'
    )

    # Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[
            {'provide_odom_frame': LaunchConfiguration('provide_odom_frame')},
            {'expected_sensor_ids': LaunchConfiguration('expected_sensor_ids')}
        ],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'sf45b_2d.lua'
        ],
        remappings=[('scan', '/scan')],
        output='screen'
    )

    # Cartographer occupancy grid node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        arguments=[
            '-resolution', LaunchConfiguration('resolution'),
            '-publish_period_sec', LaunchConfiguration('publish_period_sec')
        ],
        output='screen'
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        lidar_port_arg,
        high_angle_limit_arg,
        low_angle_limit_arg,
        provide_odom_frame_arg,
        expected_sensor_ids_arg,
        resolution_arg,
        publish_period_arg,
        sf45b_node,
        static_transform_publisher,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])
