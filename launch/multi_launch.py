from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='udco2s_ros2',
            executable='get_data',
            name='udco2s_node'
        ),
        Node(
            package='switchbot_ros2',
            executable='get_Mater',
            name='switchbot_node'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['udp4', '--port', '8888'],
            name='micro_ros_agent_node'
        ),
        Node(
            package='jsonPub_ros2',
            executable='publish_data',
            name='json_publisher_node'
        )
    ])
