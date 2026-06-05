from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('lidar_calib')

    mode = LaunchConfiguration('mode')

    # mode 값에 따라 config 파일 자동 선택
    # params_file 을 직접 지정하면 mode 는 무시됨
    auto_config = PathJoinSubstitution([
        pkg, 'config',
        PythonExpression([
            "'params_ouster.yaml' if '", mode, "' == 'ouster' else "
            "'params_general_3m_test.yaml' if '", mode, "' == '3m_test' else "
            "'params_general.yaml'"
        ]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='general',
            description='"general" | "3m_test" (Isaac Sim 3m 간격) | "ouster" (실차)'),

        DeclareLaunchArgument(
            'params_file',
            default_value=auto_config,
            description='파라미터 YAML 경로. 지정하면 mode 인자보다 우선 적용됨.'),

        Node(
            package='lidar_calib',
            executable='lidar_calibration_node',
            name='lidar_calibration_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
