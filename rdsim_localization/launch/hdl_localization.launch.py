#############################################################################

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

import launch_ros.actions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument


def generate_launch_description():

    # Importtant, required arguments
    points_topic = LaunchConfiguration("points_topic", default="/ouster/points")
    odom_child_frame_id = LaunchConfiguration(
        "odom_child_frame_id", default="base_footprint"
    )
    imu_topic = LaunchConfiguration("imu_topic", default="/imu")
    globalmap_pcd = DeclareLaunchArgument(
        "globalmap_pcd",
        default_value="/home/kdw/slam_ws/src/glim_ros2/map.pcd",
        description="Path to the global map PCD file",
    )

    # arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # optional arguments
    use_imu = LaunchConfiguration("use_imu", default="true")
    invert_imu_acc = LaunchConfiguration("invert_imu_acc", default="false")
    invert_imu_gyro = LaunchConfiguration("invert_imu_gyro", default="false")
    use_global_localization = LaunchConfiguration(
        "use_global_localization", default="false"
    )
    enable_robot_odometry_prediction = LaunchConfiguration(
        "enable_robot_odometry_prediction", default="false"
    )
    robot_odom_frame_id = LaunchConfiguration("robot_odom_frame_id", default="odom")
    plot_estimation_errors = LaunchConfiguration(
        "plot_estimation_errors", default="false"
    )

    # optional tf arguments
    lidar_tf = Node(
        name="lidar_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0", "0", "0", "1", "odom", "base_link"],
    )

    container = ComposableNodeContainer(
        name="container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hdl_localization",
                plugin="hdl_localization::GlobalmapServerNodelet",
                name="GlobalmapServerNodelet",
                parameters=[
                    {"globalmap_pcd": LaunchConfiguration("globalmap_pcd")},
                    {"convert_utm_to_local": True},
                    {"downsample_resolution": 0.1},
                ],
            ),
            ComposableNode(
                package="hdl_localization",
                plugin="hdl_localization::HdlLocalizationNodelet",
                name="HdlLocalizationNodelet",
                remappings=[
                    ("/velodyne_points", points_topic),
                    ("/gpsimu_driver/imu_data", imu_topic),
                ],
                parameters=[
                    {"odom_child_frame_id": odom_child_frame_id},
                    {"use_imu": use_imu},
                    {"invert_acc": invert_imu_acc},
                    {"invert_gyro": invert_imu_gyro},
                    {"cool_time_duration": 2.0},
                    {
                        "enable_robot_odometry_prediction": enable_robot_odometry_prediction
                    },
                    {"robot_odom_frame_id": robot_odom_frame_id},
                    # <!-- available reg_methods: NDT_OMP, NDT_CUDA_P2D, NDT_CUDA_D2D-->
                    {"reg_method": "NDT_OMP"},
                    {"ndt_neighbor_search_method": "DIRECT7"},
                    {"ndt_neighbor_search_radius": 1.0},
                    {"ndt_resolution": 0.5},
                    {"downsample_resolution": 0.1},
                    {"specify_init_pose": True},
                    {"init_pos_x": 0.0},
                    {"init_pos_y": 0.0},
                    {"init_pos_z": 0.0},
                    {"init_ori_w": 1.0},
                    {"init_ori_x": 0.0},
                    {"init_ori_y": 0.0},
                    {"init_ori_z": 0.0},
                    {"use_global_localization": use_global_localization},
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            globalmap_pcd,
            launch_ros.actions.SetParameter(name="use_sim_time", value=True),
            lidar_tf,
            container,
        ]
    )
