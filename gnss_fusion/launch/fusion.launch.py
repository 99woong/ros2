"""
fusion.launch.py

FAST-LIO2 + Dual RTK GNSS 드리프트 보정 launch 파일
실행 순서:
  1. Static TF 발행 (odom→camera_init, body→base_link, base_link→gps)
  2. gnss_corrector_node: 3-state drift EKF [Δx,Δy,Δyaw]
       /Odometry(LIO) + /fix + /heading → /odometry/map + map→odom TF
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('gnss_fusion')

    # ============================================================
    # Launch Arguments
    # ============================================================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (bag replay)'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')


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

    # ============================================================
    # gnss_corrector_node
    # 3-state drift EKF: /Odometry + /fix + /heading → /odometry/map
    # ============================================================
    gnss_corrector = Node(
        package='gnss_fusion',
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

        # Static TF
        tf_base_to_gps,
        tf_body_to_base,
        tf_odom_to_camerainit,

        # Drift correction
        gnss_corrector,

        LogInfo(msg='=== All nodes launched ==='),
    ])