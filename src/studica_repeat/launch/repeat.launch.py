"""repeat.launch.py — 재생용 스택: 베이스(heading_hold 끔) + repeat_node.

  ros2 launch studica_repeat repeat.launch.py mission:=mission_a [sim:=true]
  이후 골 전송:  ros2 run studica_repeat send_route N1 N2 N3

heading_hold를 끄는 이유: T 세그먼트의 wz는 재생 제어기가 매 주기 계산한다 — 베이스가 0 근처에서
헤딩락을 걸면 두 제어기가 서로 싸운다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission = LaunchConfiguration('mission')
    missions_dir = LaunchConfiguration('missions_dir')
    sim = LaunchConfiguration('sim')
    params_file = LaunchConfiguration('params_file')
    repeat_params = LaunchConfiguration('repeat_params')

    args = [
        DeclareLaunchArgument('mission', default_value='mission_a'),
        DeclareLaunchArgument('missions_dir', default_value='~/studica_missions'),
        DeclareLaunchArgument('sim', default_value='false', description='true면 sim_node가 HAL 대체'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare('studica_base'), 'config', 'params.yaml'])),
        DeclareLaunchArgument(
            'repeat_params',
            default_value=PathJoinSubstitution([FindPackageShare('studica_repeat'), 'config', 'repeat_params.yaml']),
            description='재생 제어기 게인'),
    ]

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('studica_base'), 'launch', 'base.launch.py'])),
        launch_arguments={'sim': sim, 'params_file': params_file, 'heading_hold': 'false'}.items(),
    )

    repeat_node = Node(
        package='studica_repeat',
        executable='repeat_node',
        name='repeat_node',
        output='screen',
        parameters=[repeat_params, {'mission': mission, 'missions_dir': missions_dir}],
    )

    return LaunchDescription(args + [base, repeat_node])
