import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 두산 로봇 시뮬레이터 패키지 경로 찾기
    dsr_bringup2_dir = get_package_share_directory('dsr_bringup2')
    
    # 2. 시뮬레이터(RViz) 실행 설정
    # (기존 UI 코드에 있던 subprocess 내용을 여기서 ROS 표준 방식으로 실행)
    dsr_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dsr_bringup2_dir, 'launch', 'dsr_bringup2_rviz.launch.py')
        ),
        launch_arguments={
            'mode': 'virtual',
            'host': '127.0.0.1',
            'port': '12345',
            'model': 'm0609'
        }.items()
    )

    # 3. UI 노드 (Client)
    ui_node = Node(
        package='babo',
        executable='bio_ui',       # setup.py entry_points 이름
        name='ui_client',
        output='screen'
        # UI는 별도 창이 뜨므로 터미널 로그는 screen으로 설정
    )


    # 3. [분리형] 메인 노드 (전화기 3개짜리)
    main_sep = Node(package='babo', executable='main_sep', name='main_separated', output='screen')

    # 4. [분리형] 하위 노드 3총사
    sub_in = Node(package='babo', executable='sub_in', name='sub_inbound', output='screen')
    sub_out = Node(package='babo', executable='sub_out', name='sub_outbound', output='screen')
    sub_move = Node(package='babo', executable='sub_move', name='sub_transport', output='screen')

    return LaunchDescription([
        dsr_simulator,
        ui_node,
        main_sep,
        sub_in,
        sub_out,
        sub_move
    ])