import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    my_package_name = 'my_robot_pkg'         
    my_executable_name = 'obstacle'         
    
    default_map_path = '/home/shcho/ros2_ws3/src/my_robot_pkg/map/map.yaml'  
    # ------------------------------------------------------------

    # 1. Nav2 관련 경로 가져오기
    try:
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    except PackageNotFoundError:
        print("Error: 'nav2_bringup' 패키지를 찾을 수 없습니다. 설치를 확인하세요.")
        return LaunchDescription([])

    # 2. Launch Argument 설정 (터미널에서 옵션으로 변경 가능)
    # 실제 로봇이면 'false', 시뮬레이션이면 'true'
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map', default=default_map_path)
    
    # Nav2 기본 파라미터 파일 사용 (필요시 커스텀 경로로 변경)
    params_file = LaunchConfiguration('params_file', 
        default=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'))

    # 3. Localization (Map Server + AMCL) 실행
    # Nav2 패키지에 포함된 검증된 런치 파일을 불러와서 실행합니다.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',  # [중요] 노드를 자동으로 Active 상태로 전환
            'use_lifecycle_mgr': 'false' 
        }.items()
    )

    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    mppi_nav_node = Node(
        package=my_package_name,      
        executable=my_executable_name, 
        name='mppi_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 6. 실행 목록 반환
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to map yaml file to load'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        localization_launch,
        rviz_node,
        mppi_nav_node
    ])