from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument('can_device',       default_value='PCAN_USBBUS1',
                              description='PEAK PCAN device name'),
        DeclareLaunchArgument('can_baudrate',     default_value='250000',
                              description='CAN bus baudrate (RFM default 250000)'),
        DeclareLaunchArgument('node_id',          default_value='15',
                              description='RFM CAN node ID (15=front/open, 16=rear/closed)'),
        DeclareLaunchArgument('trigger_mode',     default_value='false',
                              description='false=sync-input(RFM starts), true=trigger(NS starts)'),
        DeclareLaunchArgument('read_frequency_ms',default_value='100',
                              description='CAN poll interval in milliseconds'),
        DeclareLaunchArgument('frame_id',         default_value='rfm21',
                              description='TF frame ID for published poses'),

        # ── Node ─────────────────────────────────────────────────────────────
        Node(
            package    = 'rfm21_driver',
            executable = 'rfm21_can_node',
            name       = 'rfm21_can_node',
            output     = 'screen',
            parameters = [{
                'can_device':        LaunchConfiguration('can_device'),
                'can_baudrate':      LaunchConfiguration('can_baudrate'),
                'node_id':           LaunchConfiguration('node_id'),
                'trigger_mode':      LaunchConfiguration('trigger_mode'),
                'read_frequency_ms': LaunchConfiguration('read_frequency_ms'),
                'frame_id':          LaunchConfiguration('frame_id'),
            }],
        ),
    ])
