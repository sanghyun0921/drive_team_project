import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 및 경로 설정
    pkg_name = 'my_robot_pkg' # 패키지 이름이 맞는지 확인하세요
    pkg_share = get_package_share_directory(pkg_name)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 2. 파일 경로 설정
    # 앞서 만든 yaml 파일이 해당 경로에 있어야 합니다.
    params_file = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    
    # 맵 파일 경로 (map.yaml과 map.pgm이 같은 폴더에 있어야 함)
    map_file_path = os.path.join(pkg_share, 'map', 'map.yaml')

    # 3. Launch Configuration 설정
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file_config = LaunchConfiguration('params_file')

    # 4. Argument 선언 (터미널에서 변경 가능하도록)
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false', # 실제 로봇이므로 false
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # 5. Nav2 Bringup 실행 (핵심)
    # 로봇 드라이버는 포함하지 않고, 오직 내비게이션 스택만 실행합니다.
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file_config,
            'autostart': 'true',  # 로드 후 자동으로 활성화 상태로 전환
            'use_composition': 'True' # 노드 통합으로 성능 최적화 (선택 사항)
        }.items()
    )

    # 6. RViz2 실행 (시각화)
    # Nav2 상태를 확인하고 목표 지점을 찍기 위해 필수입니다.
    # Nav2 패키지 내의 기본 설정 파일을 사용하거나 직접 만든 .rviz 파일을 지정할 수 있습니다.
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # LaunchDescription 생성
    ld = LaunchDescription()

    # 선언한 Argument 추가
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)

    # 실행할 액션 추가
    ld.add_action(start_nav2_cmd)
    ld.add_action(start_rviz_cmd)

    return ld