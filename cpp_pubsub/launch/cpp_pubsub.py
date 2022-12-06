from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    freq_arg = LaunchConfiguration('count', default='100')
    record = LaunchConfiguration('record', default='True')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen',
            condition=IfCondition(record)
        ),
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='talker',
            output='screen',
            emulate_tty=True,
            parameters=[{'freq': freq_arg}]
        ),
        Node(
            package='cpp_pubsub',
            executable='listener',
            name='listener',
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(record)
        )
    ])