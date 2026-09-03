from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Robot 1: Opens in its own terminal window
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 'ros2', 'run', 'kuka_udp_bridge_node', 'udp_bridge_node',
                '--ros-args',
                '-r', '__ns:=/robot1',
                '-p', 'robot_ip:=172.31.1.10',
                '-p', 'robot_port:=30300',
                '-p', 'client_port:=30333',
                '-p', 'network_interface:=eth0'
            ],
            output='screen'
        ),
        
        # Robot 2: Opens in a separate terminal window
        ExecuteProcess(
            cmd=[
                'gnome-terminal', '--', 'ros2', 'run', 'kuka_udp_bridge_node', 'udp_bridge_node',
                '--ros-args',
                '-r', '__ns:=/robot2',
                '-p', 'robot_ip:=172.31.1.10',
                '-p', 'robot_port:=30300',
                '-p', 'client_port:=30335',
                '-p', 'network_interface:=eth1'
            ],
            output='screen'
        )
    ])