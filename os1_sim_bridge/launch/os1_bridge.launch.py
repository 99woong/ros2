from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic',  default_value='/point_cloud'),
        DeclareLaunchArgument('output_topic', default_value='/os1_cloud_node/points'),
        DeclareLaunchArgument('frame_id',     default_value='sim_lidar'),

        Node(
            package='os1_sim_bridge',
            executable='os1_bridge_node',
            name='os1_sim_bridge',
            output='screen',
            parameters=[{
                'input_topic':  LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'frame_id':     LaunchConfiguration('frame_id'),
            }],
        ),
    ])