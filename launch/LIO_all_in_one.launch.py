import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. 雷达驱动 (包含现有的 launch 文件)
    # 注意：需要确保你的环境变量中已经 source 了 lslidar_ws
    ls_radar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('lslidar_driver'), 'launch', 'lslidar_cx_launch.py')
        ])
    )

    # 2. IMU 驱动 (直接运行节点)
    imu_node = ExecuteProcess(
        cmd=['ros2', 'run', 'imu_ros2_device', 'ybimu_driver'],
        output='screen'
    )

    # 3. FastLIO2 建图 (包含你自己的 launch 文件)
    # 替换为你实际存放 my.launch.py 的包名或路径
    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/home/lyx/Downloads/FAST_LIO2_ws/my.launch.py' 
        ])
    )

    return LaunchDescription([
        ls_radar,
        imu_node,
        fast_lio
    ])