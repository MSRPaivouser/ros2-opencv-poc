import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('poc_percepcao')
    world = os.path.join(pkg, 'worlds', 'bocbot_office.sdf')

    # Simulador Gazebo (ja inicia rodando: -r)
    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen')

    # Ponte Gazebo <-> ROS 2: imagem da camera (Gazebo -> ROS) e comando de velocidade (ROS -> Gazebo)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
    )

    # No de percepcao (acha a bola) e no de controle (decide e comanda as rodas)
    percepcao = Node(package='poc_percepcao', executable='color_detector', output='screen')
    controle = Node(
        package='poc_percepcao',
        executable='target_follower',
        output='screen',
        parameters=[{
            'kp_angular': 1.0,
            'linear_speed': 0.3,
            'search_speed': -0.6,
            'stop_area': 0.05,
        }],
    )

    return LaunchDescription([gazebo, bridge, percepcao, controle])
