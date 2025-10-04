#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
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

    # SF45B driver process
    sf45b_process = ExecuteProcess(
        cmd=['./sf45b'],
        cwd='/workspace',
        name='sf45b_driver',
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
        provide_odom_frame_arg,
        expected_sensor_ids_arg,
        resolution_arg,
        publish_period_arg,
        sf45b_process,
        static_transform_publisher,
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])
