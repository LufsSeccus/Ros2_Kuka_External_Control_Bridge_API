from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kuka_udp_bridge_node',
            executable='udp_bridge_node',
            name='kuka_bridge_2',
            namespace = 'robot1',
            output='screen'
        ),
        Node(
            package='kuka_udp_bridge_node',
            executable='udp_bridge_node',
            name='kuka_bridge_2',
            namespace = 'robot2',
            output='screen',
            parameters= [{
                'client_port': 30335 , 
                'network_interface': 'eth1'
            }]
        )
    ])