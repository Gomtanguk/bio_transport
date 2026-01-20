import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'babo'
    dsr_bringup2_dir = get_package_share_directory('dsr_bringup2')
    
    # 인자 설정
    mode_arg = DeclareLaunchArgument('mode', default_value='virtual')
    # 로컬 에뮬레이터 사용 시 127.0.0.1이 더 안정적일 수 있음
    host_arg = DeclareLaunchArgument('host', default_value='127.0.0.1')

    # 1. 두산 로봇 드라이버/에뮬레이터 실행
    dsr_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dsr_bringup2_dir, 'launch', 'dsr_bringup2_rviz.launch.py')
        ),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'host': LaunchConfiguration('host'),
            'model': 'm0609',
            'port': '12345'
        }.items()
)

    # 2. 로봇 제어 서버 (bio_sub) - 15초 후 실행
    # 에뮬레이터가 12345 포트를 완전히 열 때까지 충분히 기다림
    sub_node = TimerAction(
        period=15.0,
        actions=[Node(
            package=package_name,
            executable='bio_sub',
            name='rack_transport_action',
            output='screen'
        )]
    )

    # 3. 메인 노드 (bio_main) - 18초 후 실행
    main_node = TimerAction(
        period=18.0,
        actions=[Node(
            package=package_name,
            executable='bio_main',
            name='main_orchestrator',
            output='screen'
        )]
    )

    # 4. UI 노드 (bio_ui) - 20초 후 실행
    ui_node = TimerAction(
        period=20.0,
        actions=[Node(
            package=package_name,
            executable='bio_ui',
            name='ui_client',
            output='screen'
        )]
    )

    return LaunchDescription([
        mode_arg,
        host_arg,
        dsr_simulator,
        sub_node,
        main_node,
        ui_node
    ])