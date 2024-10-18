
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# rviz cfg finding
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    local_dir = get_package_share_directory('rdsim_localization')
    nav_dir = get_package_share_directory('rdsim_nav2')
    description_dir = get_package_share_directory('rdsim_description')

    use_rviz = LaunchConfiguration('use_rviz', default='false')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui', default='false')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=PathJoinSubstitution(
            [
                FindPackageShare('rdsim_nav2'),
                'rviz',
                'nav2.rviz'
            ]
        ))


    # Specify the actions
    rdsim_description = GroupAction([

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch',
                         'rdsim_gazebo.launch.py')),

            launch_arguments={
            'start_rviz': use_rviz,
            'use_gazebo_gui': use_gazebo_gui
        }.items()),

    ])

    # Specify the actions
    rdsim_nav2 = GroupAction([

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch',
                         'nav2_gazebo.launch.py')),

                launch_arguments={
                'rviz_config_file': rviz_config_file,
            }.items()),

    ])

    # Specify the actions
    rdsim_localization = GroupAction([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(local_dir, 'launch',
                         'hdl_localization.launch.py')),
            ),

    ])

    # Specify the actions

    ## Parameters (replace frame names in case of namespacing)
    rl_params_file = os.path.join(local_dir, "config", "dual_ekf_navsat_params.yaml")
    rdsim_gps_localization = GroupAction([

        # Navsat tranform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            output='screen',
            parameters=[rl_params_file, {"use_sim_time": True}],
            remappings=[('gps/fix', '/gps/gps/data'),
                        ('imu/data', '/imu'),
                        ('odometry/filtered', '/odom'),
                        ('odometry/gps', 'navsat_transform/navsat_odometry'),
                        ('gps/filtered', 'navsat_transform/filtered_fix')
                        ]
        ),

        # Global EKF for odom => map tranform
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='global_ekf_node',
            output='screen',
            parameters=[rl_params_file, {"use_sim_time": True}],
            remappings=[('odometry/filtered', 'global_ekf/odometry_filtered')]
        ),


    ])


    return LaunchDescription([
        rdsim_localization, #ros2 launch rdsim_localization hdl_localization.launch.py
        rdsim_nav2,  #ros2 launch rdsim_nav2 nav2_gazebo.launch.py,
        rdsim_description, #ros2 launch rdsim_description rdsim_gazebo.launch.py,
        rdsim_gps_localization
    ])
