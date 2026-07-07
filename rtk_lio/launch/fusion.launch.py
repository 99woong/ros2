"""
fusion.launch.py

실행 순서:
  1. fast_lio (mapping.launch.py) — 즉시
  2. RViz (rtk_lio.rviz)         — 3초 후
  3. TF + gnss_corrector_node    — 5초 후
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = FindPackageShare('rtk_lio')

    
    # Launch Arguments
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (bag replay)'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    
    # 1. fast_lio
    
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',  # parent context 상속 방지 (wall clock 강제)
            'rviz': 'false',          # fusion.launch에서 별도 RViz 실행
        }.items()
    )

    
    # 2. RViz
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'rtk_lio.rviz'])],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    
    # 3. RTK-LIO: Static TF + gnss_corrector_node
    

    # odom → camera_init 연결 (이름 브릿지)
    tf_odom_to_camerainit = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_odom_to_camerainit',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'odom', 'camera_init'   # odom을 camera_init의 부모로
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static TF: base_link → gps (주 안테나)
    tf_base_to_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_gps',
        arguments=[
            '-0.015',  '0.283',  '0.0',   # x y z  (실측값 교체)
            '0.0',  '0.0',  '0.0',   # yaw pitch roll
            'base_link', 'gps'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static TF: body → base_link (FAST-LIO2 내부 프레임 연결)
    tf_body_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_body_to_base',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'body', 'base_link'       # FAST-LIO2 child_frame_id 확인 후 수정
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # gnss_corrector_node
    # 3-state drift EKF: /Odometry + /fix + /heading → /odometry/map
    gnss_corrector = Node(
        package='rtk_lio',
        executable='gnss_corrector_node',
        name='gnss_corrector_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'gnss_corrector.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        LogInfo(msg='=== GNSS Fusion Launch Start ==='),

        # 1. fast_lio 즉시 시작
        LogInfo(msg='[1/3] Launching fast_lio...'),
        fast_lio_launch,

        # 2. RViz: 3초 후 시작 (fast_lio 초기화 대기)
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='[2/3] Launching RViz...'),
                rviz_node,
            ]
        ),

        # 3. RTK-LIO: 5초 후 시작 (fast_lio 완전 초기화 대기)
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg='[3/3] Launching RTK-LIO nodes...'),
                tf_base_to_gps,
                tf_body_to_base,
                tf_odom_to_camerainit,
                gnss_corrector,
                LogInfo(msg='=== All nodes launched ==='),
            ]
        ),
    ])