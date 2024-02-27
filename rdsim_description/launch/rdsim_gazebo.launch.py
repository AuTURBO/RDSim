import launch


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    # 환경 변수 설정
    gazebo_model_path = os.getenv('GAZEBO_MODEL_PATH', '')
    new_model_path = os.path.expanduser('~/ros2_ws/src/RDSim/rdsim_description/models')
    combined_gazebo_model_path = f"{gazebo_model_path}:{new_model_path}" if gazebo_model_path else new_model_path

    # GAZEBO_MODEL_PATH 환경 변수 설정
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=combined_gazebo_model_path
    )

    start_rviz = LaunchConfiguration('start_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    # urdf 파일의 경로를 설정합니다.
    default_model_dir = PathJoinSubstitution(
        [
            FindPackageShare('rdsim_description'),
            'urdf',
            'bmkbot_gazebo.urdf.xacro'
        ]
    )
    # rviz 파일의 경로를 설정합니다.
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare('rdsim_description'),
            'rviz',
            'display.rviz'
        ]
    )
    # world 파일의 경로를 설정합니다.
    world_dir= PathJoinSubstitution(
        [
            FindPackageShare('rdsim_description'),
            'worlds',
            'small_city.world'
        ]
    )
    

    # robot_state_publisher를 실행하는 노드를 설정합니다.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]

    )

    # joint_state_publisher를 실행하는 노드를 설정합니다.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    # rviz를 실행하는 노드를 설정합니다.
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
            condition=IfCondition(start_rviz)),
    
    # gazebo를 실행하여 월드를 불러옵니다.
    spawn_entity = Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'rdsim_bot', '-topic', 'robot_description', '-x', '0.5', '-y', '0.5', '-z', '0.01'],
        output='screen'
    )


    return LaunchDescription([
        set_gazebo_model_path,

        # 런치 파일에 사용할 인자들을 정의합니다.
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether execute rviz2'),

        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Start robot in Gazebo simulation'),
            
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'),

        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_dir,
            description='Absolute path to robot urdf file'),

        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=rviz_config_file,
            description='Absolute path to rviz config file'),
            
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'),

        # gazebo를 실행합니다.
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_dir],
            output='screen'),

        # 위에서 정의한 노드들을 실행합니다.
        # 로봇의 상태를 퍼블리시하는 노드
        # joint_state_publisher_node,
        # robot_state_publisher_node,
        # joint_state_publisher_gui_node,

        # 로봇을 gazebo에 스폰하는 노드
        # spawn_entity,
        # rviz_node,
    ])