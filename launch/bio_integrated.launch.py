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
    
    # =========================================================
    # 1. 런치 인자 설정 (터미널에서 변경 가능하도록)
    # =========================================================
    # 기본 모드: real
    mode_arg = DeclareLaunchArgument('mode', default_value='virtual')
    host_arg = DeclareLaunchArgument('host', default_value='127.0.0.1')
    
    # mode_arg = DeclareLaunchArgument('mode', default_value='real')
    # host_arg = DeclareLaunchArgument('host', default_value='192.168.1.100')
                       
    # [중요] 아까 테스트 성공했던 파라미터를 여기서도 쓸 수 있게 추가
    dry_run_arg = DeclareLaunchArgument(
        'dry_run', 
        default_value='False', 
        description='Set to True to bypass robot connection'
    )
    skip_probe_arg = DeclareLaunchArgument(
        'skip_probe', 
        default_value='True', 
        description='Set to True to skip sensor check in virtual mode'
    )

    # =========================================================
    # 2. 노드 실행 정의
    # =========================================================
    
    # (A) 두산 로봇 에뮬레이터/RViz 실행
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

    # (B) 로봇 제어 서버 (bio_sub) - 15초 후 실행
    # [수정] parameters 옵션을 통해 dry_run/skip_probe 값을 전달
    sub_node = TimerAction(
        period=15.0,
        actions=[Node(
            package=package_name,
            executable='bio_sub',
            name='rack_transport_action',
            output='screen',
            parameters=[{
                'dry_run': LaunchConfiguration('dry_run'),
                'skip_probe': LaunchConfiguration('skip_probe')
            }]
        )]
    )

    # (C) 메인 오케스트레이터 (bio_main) - 18초 후 실행
    main_node = TimerAction(
        period=18.0,
        actions=[Node(
            package=package_name,
            executable='bio_main',
            name='main_orchestrator',
            output='screen'
        )]
    )

    # (D) UI 노드 (bio_ui) - 20초 후 실행
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
        # 인자 등록
        mode_arg,
        host_arg,
        dry_run_arg,
        skip_probe_arg,
        # 실행 그룹
        dsr_simulator,
        sub_node,
        main_node,
        ui_node
    ])