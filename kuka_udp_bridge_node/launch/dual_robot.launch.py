from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'xterm', '-e', 'ros2', 'run', 'kuka_udp_bridge_node', 'udp_bridge_node',
                '--ros-args', '-r', '__ns:=/robot1',
                '-p', 'robot_ip:=172.31.1.10', '-p', 'client_port:=30333', '-p', 'network_interface:=eth0'
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                'xterm', '-e', 'ros2', 'run', 'kuka_udp_bridge_node', 'udp_bridge_node',
                '--ros-args', '-r', '__ns:=/robot2',
                '-p', 'robot_ip:=172.31.1.10', '-p', 'client_port:=30335', '-p', 'network_interface:=eth1'
            ],
            output='screen'
        )
    ])