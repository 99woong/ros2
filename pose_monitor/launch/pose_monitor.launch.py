import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import OpaqueFunction, EmitEvent
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('pose_monitor')
    param_file = os.path.join(pkg_dir, 'config', 'monitor_params.yaml')
    
    pose_monitor_node = Node(
        package='pose_monitor',
        executable='pose_monitor_node',
        name='pose_monitor_node',
        output='screen',
        parameters=[param_file], # YAML 파일 로드
        emulate_tty=True,       
        # pose_monitor 노드가 종료되면(Exit), 런치 세션에 종료 이벤트(Shutdown)를 보냄
        on_exit=EmitEvent(event=Shutdown()), 
        respawn=False
    )
    
    vloc_receiver_node = Node(
        package='vloc_receiver',
        executable='vloc_receiver_node',
        name='vloc_receiver',
        output='screen',
        emulate_tty=True,
    )

    # gls100_node = Node(
    #     package='gls100_ros2',
    #     executable='gls100_node',
    #     name='gls100_node',
    #     output='screen',
    #     emulate_tty=True,
    # )

    gls100_node = Node(
        package='gls100_driver',
        executable='generic_can_node',
        # name='gls100_node',
        output='screen',
        emulate_tty=True,
    )


    return LaunchDescription([
        vloc_receiver_node,
        # gls100_node,
        gls100_node,
        pose_monitor_node, # 종료 감지를 위해 on_exit가 설정된 노드
    ])