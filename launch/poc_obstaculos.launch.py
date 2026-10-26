import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('poc_percepcao')
    world = os.path.join(pkg, 'worlds', 'bocbot_sala.sdf')

    # Simulador Gazebo (ja inicia rodando: -r)
    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen')

    # Ponte Gazebo <-> ROS 2:
    #   camera e lidar: Gazebo -> ROS 2   |   comando de velocidade: ROS 2 -> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen',
    )

    # Percepcao (camera: acha a bola) e controle (decide e desvia das paredes com o lidar)
    percepcao = Node(package='poc_percepcao', executable='color_detector', output='screen')
    controle = Node(package='poc_percepcao', executable='navigator', output='screen')

    return LaunchDescription([gazebo, bridge, percepcao, controle])
