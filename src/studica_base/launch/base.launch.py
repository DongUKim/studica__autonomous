"""studica_base base.launch.py

실기:  ros2 launch studica_base base.launch.py
시뮬:  ros2 launch studica_base base.launch.py sim:=true      (studica_repeat sim_node가 HAL을 대체)
재생:  ros2 launch studica_base base.launch.py heading_hold:=false  (repeat 제어기가 wz를 직접 낸다)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('studica_base')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    params_file = LaunchConfiguration('params_file')
    sim = LaunchConfiguration('sim')
    heading_hold = LaunchConfiguration('heading_hold')

    args = [
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='studica_control용 params.yaml'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='true면 하드웨어 대신 studica_repeat sim_node 실행'),
        DeclareLaunchArgument('heading_hold', default_value='true',
                              description='wz 명령 0일 때 IMU 헤딩 유지. 플랜B 재생 시 false'),
    ]

    # FindPackageShare는 지연 평가라 sim:=true(하드웨어 미포함)일 때 studica_control이
    # 설치돼 있지 않아도 launch 파싱이 실패하지 않는다
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('studica_control'),
                                  'launch', 'studica_launch.py'])),
        launch_arguments={'params_file': params_file}.items(),
        condition=UnlessCondition(sim),
    )

    sim_node = Node(
        package='studica_repeat',
        executable='sim_node',
        name='sim_node',
        output='screen',
        condition=IfCondition(sim),
    )

    base_node = Node(
        package='studica_base',
        executable='base_node',
        name='base_node',
        output='screen',
        parameters=[{
            'heading_hold': PythonExpression(["'", heading_hold, "'.lower() == 'true'"]),
        }],
    )

    return LaunchDescription(args + [hardware, sim_node, base_node])
