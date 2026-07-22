import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ouster_pkg = get_package_share_directory('ouster_ros')
    this_pkg   = get_package_share_directory('dual_lidar_fastlio')

    # components.yaml 에서 기본 활성화 값 읽기
    components_yaml = os.path.join(this_pkg, 'config', 'components.yaml')
    with open(components_yaml, 'r') as f:
        comp = yaml.safe_load(f)

    def bstr(key):
        return 'true' if comp.get(key, True) else 'false'

    # ── 공통 인자 ─────────────────────────────────────────────────────────
    lidar_mode   = LaunchConfiguration('lidar_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_lidar_mode = DeclareLaunchArgument(
        'lidar_mode', default_value='1024x10',
        description='Ouster lidar mode (e.g. 512x10, 1024x10, 1024x20)')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    # ── 컴포넌트 활성화 인자 (기본값 = components.yaml, CLI로 override 가능) ──
    declare_ouster_1      = DeclareLaunchArgument('ouster_1',        default_value=bstr('ouster_1'),        description='Enable Ouster driver 1')
    declare_ouster_2      = DeclareLaunchArgument('ouster_2',        default_value=bstr('ouster_2'),        description='Enable Ouster driver 2')
    declare_fast_lio_1    = DeclareLaunchArgument('fast_lio_1',      default_value=bstr('fast_lio_1'),      description='Enable FAST-LIO2 instance 1')
    declare_fast_lio_2    = DeclareLaunchArgument('fast_lio_2',      default_value=bstr('fast_lio_2'),      description='Enable FAST-LIO2 instance 2')
    declare_rviz_1        = DeclareLaunchArgument('fast_lio_1_rviz', default_value=bstr('fast_lio_1_rviz'), description='Enable RViz2 for FAST-LIO2 instance 1')
    declare_rviz_2        = DeclareLaunchArgument('fast_lio_2_rviz', default_value=bstr('fast_lio_2_rviz'), description='Enable RViz2 for FAST-LIO2 instance 2')

    # ── Ouster 1 (192.168.10.11) ─────────────────────────────────────────
    ouster1 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(ouster_pkg, 'launch', 'sensor.launch.xml')
        ),
        launch_arguments={
            'ouster_ns':       'ouster1',
            'sensor_hostname': '192.168.10.11',
            'lidar_mode':      lidar_mode,
            'viz':             'false',
            'sensor_frame':    'os_sensor_1',
            'lidar_frame':     'os_lidar_1',
            'imu_frame':       'os_imu_1',
            'lidar_port':      '7502',
            'imu_port':        '7503',
        }.items(),
        condition=IfCondition(LaunchConfiguration('ouster_1')),
    )

    # ── Ouster 2 (192.168.20.11) ─────────────────────────────────────────
    ouster2 = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(ouster_pkg, 'launch', 'sensor.launch.xml')
        ),
        launch_arguments={
            'ouster_ns':       'ouster2',
            'sensor_hostname': '192.168.20.11',
            'lidar_mode':      lidar_mode,
            'viz':             'false',
            'sensor_frame':    'os_sensor_2',
            'lidar_frame':     'os_lidar_2',
            'imu_frame':       'os_imu_2',
            'lidar_port':      '7512',
            'imu_port':        '7513',
        }.items(),
        condition=IfCondition(LaunchConfiguration('ouster_2')),
    )

    # ── FAST-LIO2 instance 1 ─────────────────────────────────────────────
    fastlio1 = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping_1',
        namespace='fast_lio1',
        parameters=[
            os.path.join(this_pkg, 'config', 'ouster64_1.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/Odometry',              '/Odometry1'),
            ('/cloud_registered',      '/cloud_registered1'),
            ('/cloud_registered_body', '/cloud_registered_body1'),
            ('/cloud_effected',        '/cloud_effected1'),
            ('/Laser_map',             '/Laser_map1'),
            ('/path',                  '/path1'),
        ],
        condition=IfCondition(LaunchConfiguration('fast_lio_1')),
        output='screen',
    )

    # ── FAST-LIO2 instance 2 ─────────────────────────────────────────────
    fastlio2 = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping_2',
        namespace='fast_lio2',
        parameters=[
            os.path.join(this_pkg, 'config', 'ouster64_2.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/Odometry',              '/Odometry2'),
            ('/cloud_registered',      '/cloud_registered2'),
            ('/cloud_registered_body', '/cloud_registered_body2'),
            ('/cloud_effected',        '/cloud_effected2'),
            ('/Laser_map',             '/Laser_map2'),
            ('/path',                  '/path2'),
        ],
        condition=IfCondition(LaunchConfiguration('fast_lio_2')),
        output='screen',
    )

    # ── RViz2 for FAST-LIO2 instance 1 ───────────────────────────────────
    rviz1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_fastlio1',
        arguments=['-d', os.path.join(this_pkg, 'rviz', 'fastlio_1.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('fast_lio_1_rviz')),
        output='screen',
    )

    # ── RViz2 for FAST-LIO2 instance 2 ───────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_fastlio2',
        arguments=['-d', os.path.join(this_pkg, 'rviz', 'fastlio_2.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('fast_lio_2_rviz')),
        output='screen',
    )

    return LaunchDescription([
        declare_lidar_mode,
        declare_use_sim_time,
        declare_ouster_1,
        declare_ouster_2,
        declare_fast_lio_1,
        declare_fast_lio_2,
        declare_rviz_1,
        declare_rviz_2,
        ouster1,
        ouster2,
        fastlio1,
        fastlio2,
        rviz1,
        rviz2,
    ])
