"""teach.launch.py — 티칭용 베이스 스택(HAL 또는 sim + base_node, heading_hold 켬).

  ros2 launch studica_repeat teach.launch.py mission:=mission_a [sim:=true]

teach_node(키보드)는 tty가 필요해 launch로 띄우지 않는다. 별도 터미널에서:
  ros2 run studica_repeat teach_node --ros-args -p mission:=mission_a
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission = LaunchConfiguration('mission')
    sim = LaunchConfiguration('sim')
    params_file = LaunchConfiguration('params_file')

    args = [
        DeclareLaunchArgument('mission', default_value='mission_a', description='미션 이름 (안내용)'),
        DeclareLaunchArgument('sim', default_value='false', description='true면 sim_node가 HAL 대체'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare('studica_base'), 'config', 'params.yaml']),
            description='studica_control params.yaml'),
        DeclareLaunchArgument('missions_dir', default_value='~/studica_missions',
                              description='레그 저장 루트 (teach_node에 -p missions_dir 로 넘길 것)'),
    ]

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('studica_base'), 'launch', 'base.launch.py'])),
        launch_arguments={'sim': sim, 'params_file': params_file, 'heading_hold': 'true'}.items(),
    )

    hint = LogInfo(msg=['teach: 다른 터미널에서  ros2 run studica_repeat teach_node --ros-args -p mission:=',
                        mission, '  (tty 필요, ssh -t)'])

    return LaunchDescription(args + [base, hint])
