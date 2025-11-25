from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 使用 D435i 的 stereo + IMU 配置
    config_file = os.path.join(
        get_package_share_directory('vins_fusion_ros2'),
        'config',
        'realsense_d435i',
        'realsense_stereo_imu_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='vins_fusion_ros2',
            executable='vins_fusion_ros2_node',
            name='vins_fusion_ros2_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': False},      # D435 用真实时间，改为 False
                {'config_file': config_file}, # 使用 d435i 配置文件
            ],
        )
    ])

