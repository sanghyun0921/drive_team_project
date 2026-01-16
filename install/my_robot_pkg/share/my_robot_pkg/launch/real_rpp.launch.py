import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 패키지 이름 설정 (사용자의 패키지 이름으로 수정 필요)
    pkg_name = 'my_robot_pkg' 
    pkg_share = get_package_share_directory(pkg_name)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 2. 파일 경로 설정
    # (앞서 작성한 RPP 설정이 담긴 YAML 파일이어야 합니다)
    params_file = os.path.join(pkg_share, 'params', 'rpp_param1.yaml')
    map_file_path = os.path.join(pkg_share, 'map', '/home/teamone/ros2_ws3/src/my_robot_pkg/map/map_202.yaml')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 3. Launch Configuration 변수 설정
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file_config = LaunchConfiguration('params_file')

    # 4. Arguments 선언
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false', 
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # 5. Nav2 Bringup 실행
    # (Lifecycle Manager, Map Server, AMCL, Controller Server 등을 모두 실행)
    # 여기서 로드되는 params_file에 RPP(Regulated Pure Pursuit) 설정이 들어있어야 합니다.
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file_config,
            'autostart': 'true',  
            'use_composition': 'True' 
        }.items()
    )

    # 6. RViz 실행
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 7. [핵심] 커스텀 A* + RPP 제어 노드 실행
    # setup.py의 entry_points에 등록한 이름을 executable에 적어주세요.
    # 예: 'a_star_rpp'
    start_planner_node_cmd = Node(
        package=pkg_name,
        executable='a_star_rpp',  # setup.py에서 설정한 진입점 이름
        name='a_star_rpp_planner', # 노드 이름
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 8. Launch Description 구성
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(start_nav2_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_planner_node_cmd) # 파이썬 제어 노드 추가

    return ld