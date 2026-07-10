"""
一键启动：Gazebo 仿真 + FAST-LIO2 建图 + Nav2 自主导航

用法：
  ros2 launch fast_lio bringup_all.launch.py

按需覆盖参数：
  ros2 launch fast_lio bringup_all.launch.py \
    world:=turtlebot3_world \
    config_file:=my_lidar.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


# ======================== 默认路径（编译期常量） ========================
DEFAULT_URDF_PATH = '/home/lyx/turtlebot3_ws/turtlebot3_waffle_pi.urdf'


def generate_launch_description():
    package_dir = get_package_share_directory('fast_lio')

    # ======================== 可配置参数 ========================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # world 参数：传入 world 文件名（含扩展名，如 turtlebot3_world.world）
    world_file = LaunchConfiguration('world', default='turtlebot3_world.world')
    config_file = LaunchConfiguration('config_file', default='my_lidar.yaml')
    rviz_use = LaunchConfiguration('rviz', default='true')
    nav2_use = LaunchConfiguration('nav2', default='true')

    # 读取 URDF 内容（编译期）
    with open(DEFAULT_URDF_PATH, 'r') as f:
        robot_desc = f.read()

    # ======================== 1. Gazebo 仿真环境 ========================
    world_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
    )
    gazebo_world = PathJoinSubstitution([world_path, world_file])

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gzserver.launch.py'
            )
        ),
        launch_arguments={'world': gazebo_world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            )
        ),
    )

    # ======================== 2. Robot State Publisher ========================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }],
    )

    # ======================== 3. 在 Gazebo 中生成机器人 ========================
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', DEFAULT_URDF_PATH,
            '-x', '0.0', '-y', '0.0', '-z', '0.01'
        ],
        output='screen',
    )

    # ======================== 4. FAST-LIO2 建图 ========================
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('fast_lio'),
                'config', config_file
            ]),
            {'use_sim_time': use_sim_time}
        ],
        output='screen',
    )

    # ======================== 5. Odometry 桥接 ========================
    odom_bridge = Node(
        package='fast_lio',
        executable='odom_bridge.py',
        name='odom_bridge',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': 'base_footprint',
            'source_frame': 'camera_init',
            'source_child_frame': 'body',
            'output_odom_topic': '/odom',
            'project_to_2d': True,
            'publish_map_to_source': True,
        }],
        output='screen',
    )

    # ======================== 6. Nav2 导航栈 ========================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'navigation.launch.py')
        ),
        condition=IfCondition(nav2_use),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # ======================== 7. RViz2 ========================
    rviz_config = PathJoinSubstitution([
        package_dir, 'rviz_cfg', 'my.rviz'
    ])
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_use),
        output='screen',
    )

    # ======================== 组装 ========================
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='turtlebot3_world.world'),
        DeclareLaunchArgument('config_file', default_value='my_lidar.yaml'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav2', default_value='true'),

        # 启动
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
        fast_lio,
        odom_bridge,
        nav2_launch,
        rviz2,
    ])
