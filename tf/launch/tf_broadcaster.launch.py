from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def generate_launch_description():

    def launch_nodes(context, *args, **kwargs):
        return [
            # map -> odom
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='map_to_odom_broadcaster',
                arguments=['--x', '0', '--y', '0', '--z', '0',
                           '--roll', '0', '--pitch', '0', '--yaw', '0',
                           '--frame-id', 'map', '--child-frame-id', 'odom'],
                output='screen'
            ),
            # odom -> base_link
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_to_base_broadcaster',
                arguments=['--x', '0', '--y', '0', '--z', '0',
                           '--roll', '0', '--pitch', '0', '--yaw', '0',
                           '--frame-id', 'odom', '--child-frame-id', 'base_link'],
                output='screen'
            ),
            # base_link -> laser_link
            Node(
                package='tf2_ros',
                executable='static_transform_broadcaster',
                name='base_to_laser_broadcaster',
                arguments=['--x', '0', '--y', '0', '--z', '0',
                           '--roll', '0', '--pitch', '0', '--yaw', '0',
                           '--frame-id', 'base_link', '--child-frame-id', 'laser_link'],
                output='screen'
            )
        ]

    return LaunchDescription([
        OpaqueFunction(function=launch_nodes)
    ])
