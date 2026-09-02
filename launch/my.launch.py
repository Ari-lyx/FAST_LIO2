import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(package_path, 'rviz_cfg', 'my.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='ls_lidar.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ======================== Odom 桥接 (FAST-LIO → Nav2 TF) ========================
    odom_bridge = Node(
        package='fast_lio',
        executable='odom_bridge.py',
        name='odom_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        output='screen',
    )

    # ======================== 点云滤波 (去地面, 降采样, → /costmap/points) ========================
    pointcloud_filter = Node(
        package='fast_lio',
        executable='nav2_pointcloud_filter.py',
        name='nav2_pointcloud_filter',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/cloud_registered_body',
            'output_topic': '/costmap/points',
            'min_range': 0.15,
            'max_range': 15.0,
            'voxel_size': 0.05,
            'ground_distance': 0.08,
            'min_obstacle_height': 0.18,
            'max_obstacle_height': 1.8,
        }],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    ld.add_action(odom_bridge)
    ld.add_action(pointcloud_filter)
    ld.add_action(rviz_node)

    return ld
