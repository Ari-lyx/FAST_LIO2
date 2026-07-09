import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('fast_lio')

    # ---- 参数 ----
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(package_dir, 'config', 'nav2_params.yaml')
    )

    # Nav2 navigation_launch.py（仅启动导航栈，不含 AMCL/slam_toolbox）
    # 定位由 FAST-LIO2 + odom_bridge 提供
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_nav_launch = os.path.join(
        nav2_bringup_dir, 'launch', 'navigation_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Nav2 params file path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_nav_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
                'use_composition': 'False',
            }.items(),
        ),
    ])
