from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Robot 1 Window (Black background, Green text)
        ExecuteProcess(
            cmd=[
                'xterm', '-T', 'KUKA Robot 1 Bridge (eth0)', 
                '-bg', 'black', '-fg', 'green', '-e', 
                'ros2', 'run', 'kuka_udp_bridge_node', 'udp_bridge_node',
                '--ros-args', '-r', '__ns:=/robot1',
                '-p', 'robot_ip:=172.31.1.10', 
                '-p', 'client_port:=30333', 
                '-p', 'network_interface:=eth0'
            ],
            output='screen'
        ),
        
        # Robot 2 Window (Dark blue background, Yellow text)
        ExecuteProcess(
            cmd=[
                'xterm', '-T', 'KUKA Robot 2 Bridge (eth1)', 
                '-bg', 'midnightblue', '-fg', 'yellow', '-e', 
                'ros2', 'run', 'kuka_udp_bridge_node', 'udp_bridge_node',
                '--ros-args', '-r', '__ns:=/robot2',
                '-p', 'robot_ip:=172.31.1.10', 
                '-p', 'client_port:=30335', 
                '-p', 'network_interface:=eth1'
            ],
            output='screen'
        )
    ])