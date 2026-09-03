"""validate.launch.py — repeat.launch.py + validate_node (전 레그 연속 재생 → tracking.csv, report.txt).

  ros2 launch studica_repeat validate.launch.py mission:=mission_a [sim:=true] [legs:=N1__N2,N2__N3]

validate_node가 끝나면 launch 전체가 종료된다(on_exit 없이 노드 종료만 — Ctrl-C로 나머지 정리).
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

    args = [
        DeclareLaunchArgument('mission', default_value='mission_a'),
        DeclareLaunchArgument('missions_dir', default_value='~/studica_missions'),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([FindPackageShare('studica_base'), 'config', 'params.yaml'])),
        DeclareLaunchArgument('legs', default_value='all', description='"all" 또는 "N1__N2,N2__N3"'),
        DeclareLaunchArgument('out_dir', default_value='', description='비우면 <missions_dir>/<mission>/validation'),
        DeclareLaunchArgument('speed_scale', default_value='1.0'),
    ]

    repeat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('studica_repeat'), 'launch', 'repeat.launch.py'])),
        launch_arguments={'mission': mission, 'missions_dir': missions_dir,
                          'sim': sim, 'params_file': params_file}.items(),
    )

    validate_node = Node(
        package='studica_repeat',
        executable='validate_node',
        name='validate_node',
        output='screen',
        parameters=[{
            'mission': mission,
            'missions_dir': missions_dir,
            'legs': LaunchConfiguration('legs'),
            'out_dir': LaunchConfiguration('out_dir'),
            'speed_scale': LaunchConfiguration('speed_scale'),
        }],
    )

    return LaunchDescription(args + [repeat, validate_node])
