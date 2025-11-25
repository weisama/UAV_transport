from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vrpn_relative_pose',
            executable='vrpn_relative_pose_node',
            name='vrpn_relative_pose',
            parameters=[{'this_uav': 't1'}]  # 修改为 t1 或 t2
        )
    ])
