// hdl localizaton ROS2 코드 1
#include <mutex>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/srv/empty.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include <hdl_localization/msg/scan_matching_status.hpp>
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>

using namespace std;

namespace hdl_localization {

class HdlLocalizationNodelet : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  HdlLocalizationNodelet(const rclcpp::NodeOptions& options) : Node("hdl_localization", options) {
    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    robot_odom_frame_id = declare_parameter<std::string>("robot_odom_frame_id", "robot_odom");
    odom_child_frame_id = declare_parameter<std::string>("odom_child_frame_id", "base_link");
    send_tf_transforms = declare_parameter<bool>("send_tf_transforms", true);
    cool_time_duration = declare_parameter<double>("cool_time_duration", 0.5);
    reg_method = declare_parameter<std::string>("reg_method", "NDT_OMP");
    ndt_neighbor_search_method = declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT7");
    ndt_neighbor_search_radius = declare_parameter<double>("ndt_neighbor_search_radius", 2.0);
    ndt_resolution = declare_parameter<double>("ndt_resolution", 1.0);
    enable_robot_odometry_prediction = declare_parameter<bool>("enable_robot_odometry_prediction", false);

    use_imu = declare_parameter<bool>("use_imu", true);
    invert_acc = declare_parameter<bool>("invert_acc", false);
    invert_gyro = declare_parameter<bool>("invert_gyro", false);
    if (use_imu) {
      RCLCPP_INFO(get_logger(), "enable imu-based prediction");
      imu_sub = create_subscription<sensor_msgs::msg::Imu>("/gpsimu_driver/imu_data", 256, std::bind(&HdlLocalizationNodelet::imu_callback, this, std::placeholders::_1));
    }
    points_sub = create_subscription<sensor_msgs::msg::PointCloud2>("/ouster/points", 5, std::bind(&HdlLocalizationNodelet::points_callback, this, std::placeholders::_1));

    auto latch_qos = rclcpp::QoS(1).transient_local();
    globalmap_sub =
      create_subscription<sensor_msgs::msg::PointCloud2>("/globalmap", latch_qos, std::bind(&HdlLocalizationNodelet::globalmap_callback, this, std::placeholders::_1));

    initialpose_sub =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 8, std::bind(&HdlLocalizationNodelet::initialpose_callback, this, std::placeholders::_1));

    pose_pub = create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
    aligned_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", 5);
    status_pub = create_publisher<msg::ScanMatchingStatus>("/status", 5);

    // global localization
    use_global_localization = declare_parameter<bool>("use_global_localization", true);
    if (use_global_localization) {
      RCLCPP_INFO_STREAM(get_logger(), "wait for global localization services");
      set_global_map_service = create_client<hdl_global_localization::srv::SetGlobalMap>("/hdl_global_localization/set_global_map");
      query_global_localization_service = create_client<hdl_global_localization::srv::QueryGlobalLocalization>("/hdl_global_localization/query");
      while (!set_global_map_service->wait_for_service(std::chrono::milliseconds(1000))) {
        RCLCPP_WARN(get_logger(), "Waiting for SetGlobalMap service");
        if (!rclcpp::ok()) {
          return;
        }
      }
      while (!query_global_localization_service->wait_for_service(std::chrono::milliseconds(1000))) {
        RCLCPP_WARN(get_logger(), "Waiting for QueryGlobalLocalization service");
        if (!rclcpp::ok()) {
          return;
        }
      }

      relocalize_server = create_service<std_srvs::srv::Empty>("/relocalize", std::bind(&HdlLocalizationNodelet::relocalize, this, std::placeholders::_1, std::placeholders::_2));
    }
    initialize_params();
  }

private:
  pcl::Registration<PointT, PointT>::Ptr create_registration() {
    if (reg_method == "NDT_OMP") {
      RCLCPP_INFO(get_logger(), "NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());
      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      if (ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method == "KDTREE") {
          RCLCPP_INFO(get_logger(), "search_method KDTREE is selected");
        } else {
          RCLCPP_WARN(get_logger(), "invalid search method was given");
          RCLCPP_WARN(get_logger(), "default method is selected (KDTREE)");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    }

    // else if(reg_method.find("NDT_CUDA") != std::string::npos) {
    //   RCLCPP_INFO(get_logger(), "NDT_CUDA is selected");
    //   boost::shared_ptr<fast_gicp::NDTCuda<PointT, PointT>> ndt(new fast_gicp::NDTCuda<PointT, PointT>);
    //   ndt->setResolution(ndt_resolution);

    //   if(reg_method.find("D2D") != std::string::npos) {
    //     ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
    //   } else if (reg_method.find("P2D") != std::string::npos) {
    //     ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
    //   }

    //   if (ndt_neighbor_search_method == "DIRECT1") {
    //     RCLCPP_INFO(get_logger(), "search_method DIRECT1 is selected");
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
    //   } else if (ndt_neighbor_search_method == "DIRECT7") {
    //     RCLCPP_INFO(get_logger(), "search_method DIRECT7 is selected");
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
    //   } else if (ndt_neighbor_search_method == "DIRECT_RADIUS") {
    //     RCLCPP_INFO_STREAM(get_logger(), "search_method DIRECT_RADIUS is selected : " << ndt_neighbor_search_radius);
    //     ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, ndt_neighbor_search_radius);
    //   } else {
    //     RCLCPP_WARN(get_logger(), "invalid search method was given");
    //   }
    //   return ndt;
    // }

    RCLCPP_ERROR_STREAM(get_logger(), "unknown registration method:" << reg_method);
    return nullptr;
  }

  void initialize_params() {
    // intialize scan matching method
    double downsample_resolution = declare_parameter<double>("downsample_resolution", 0.1);
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    downsample_filter = voxelgrid;

    RCLCPP_INFO(get_logger(), "create registration method for localization");
    registration = create_registration();

    // global localization
    RCLCPP_INFO(get_logger(), "create registration method for fallback during relocalization");
    relocalizing = false;
    delta_estimater.reset(new DeltaEstimater(create_registration()));

    // initialize pose estimator
    bool specify_init_pose = declare_parameter<bool>("specify_init_pose", true);
    if (specify_init_pose) {
      RCLCPP_INFO(get_logger(), "initialize pose estimator with specified parameters!!");
      pose_estimator.reset(new hdl_localization::PoseEstimator(
        registration,
        get_clock()->now(),
        Eigen::Vector3f(declare_parameter<double>("init_pos_x", 0.0), declare_parameter<double>("init_pos_y", 0.0), declare_parameter<double>("init_pos_z", 0.0)),
        Eigen::Quaternionf(
          declare_parameter<double>("init_ori_w", 1.0),
          declare_parameter<double>("init_ori_x", 0.0),
          declare_parameter<double>("init_ori_y", 0.0),
          declare_parameter<double>("init_ori_z", 0.0)),
        cool_time_duration));
    }
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
    // RCLCPP_INFO(get_logger(), "----------------");
    // RCLCPP_INFO(get_logger(), "imu_callback");
    // RCLCPP_INFO(get_logger(), "----------------");
    std::lock_guard<std::mutex> lock(imu_data_mutex);
    imu_data.push_back(imu_msg);
  }

  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    // RCLCPP_INFO(get_logger(), "");
    // RCLCPP_INFO(get_logger(), "points_callback");
    // RCLCPP_INFO(get_logger(), "");

    std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex);
    if (!pose_estimator) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "waiting for initial pose input!!");
      return;
    }

    if (!globalmap) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0, "globalmap has not been received!!");
      return;
    }

    const auto& stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    // point_msg의 sensor_msg/pointCloud2 type을 pcl_cloud type으로 형변환
    // sensor_msg/pointCloud2 -> pcl::PointCloud<PointT>
    pcl::fromROSMsg(*points_msg, *pcl_cloud);

    if (pcl_cloud->empty()) {
      RCLCPP_ERROR(get_logger(), "cloud is empty!!");
      return;
    }

    // transform pointcloud into odom_child_frame_id
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if (!pcl_ros::transformPointCloud(odom_child_frame_id, *pcl_cloud, *cloud, *tf_buffer)) {
      RCLCPP_ERROR(get_logger(), "point cloud cannot be transformed into target frame!!");
      return;
    }

    auto filtered = downsample(cloud);
    last_scan = filtered;

    if (relocalizing) {
      delta_estimater->add_frame(filtered);
    }

    Eigen::Matrix4f before = pose_estimator->matrix();

    // predict
    if (!use_imu) {
      pose_estimator->predict(stamp);
    } else {
      std::lock_guard<std::mutex> lock(imu_data_mutex);
      // RCLCPP_INFO(get_logger(),"imu size is : %d ", imu_data.size());
      auto imu_iter = imu_data.begin();
      for (imu_iter; imu_iter != imu_data.end(); imu_iter++) {
        if (rclcpp::Time(stamp) < rclcpp::Time((*imu_iter)->header.stamp)) {
          break;
        }
        const auto& acc = (*imu_iter)->linear_acceleration;
        const auto& gyro = (*imu_iter)->angular_velocity;
        double acc_sign = invert_acc ? -1.0 : 1.0;
        double gyro_sign = invert_gyro ? -1.0 : 1.0;
        pose_estimator->predict((*imu_iter)->header.stamp, acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z), gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
      }
      imu_data.erase(imu_data.begin(), imu_iter);
    }

    // odometry-based prediction
    rclcpp::Time last_correction_time = pose_estimator->last_correction_time();
    if (enable_robot_odometry_prediction && last_correction_time != rclcpp::Time((int64_t)0, get_clock()->get_clock_type())) {
      geometry_msgs::msg::TransformStamped odom_delta;
      if (tf_buffer->canTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, rclcpp::Duration(std::chrono::milliseconds(100)))) {
        odom_delta =
          tf_buffer->lookupTransform(odom_child_frame_id, last_correction_time, odom_child_frame_id, stamp, robot_odom_frame_id, rclcpp::Duration(std::chrono::milliseconds(0)));
      } else if (tf_buffer->canTransform(
                   odom_child_frame_id,
                   last_correction_time,
                   odom_child_frame_id,
                   rclcpp::Time((int64_t)0, get_clock()->get_clock_type()),
                   robot_odom_frame_id,
                   rclcpp::Duration(std::chrono::milliseconds(0)))) {
        odom_delta = tf_buffer->lookupTransform(
          odom_child_frame_id,
          last_correction_time,
          odom_child_frame_id,
          rclcpp::Time((int64_t)0, get_clock()->get_clock_type()),
          robot_odom_frame_id,
          rclcpp::Duration(std::chrono::milliseconds(0)));
      }

      if (odom_delta.header.stamp == rclcpp::Time((int64_t)0, get_clock()->get_clock_type())) {
        RCLCPP_WARN_STREAM(get_logger(), "failed to look up transform between " << cloud->header.frame_id << " and " << robot_odom_frame_id);
      } else {
        Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
        pose_estimator->predict_odom(delta.cast<float>().matrix());
      }
    }

    // 문제가 되는 구문
    // correct
    auto aligned = pose_estimator->correct(stamp, filtered);

    if (aligned_pub->get_subscription_count()) {
      aligned->header.frame_id = "map";
      aligned->header.stamp = cloud->header.stamp;
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_pub->publish(aligned_msg);
    }

    if (status_pub->get_subscription_count()) {
      publish_scan_matching_status(points_msg->header, aligned);
    }

    publish_odometry(points_msg->header.stamp, pose_estimator->matrix());

    // RCLCPP_INFO(get_logger(), "");
    // RCLCPP_INFO(get_logger(), "----------finish points callback------------");
    // RCLCPP_INFO(get_logger(), "");
  }

  /**
   * @brief callback for globalmap input
   * @param points_msg
   */
  void globalmap_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    RCLCPP_INFO(get_logger(), "");
    RCLCPP_INFO(get_logger(), "globalmap received!");
    RCLCPP_INFO(get_logger(), "");

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    // pcl::PointCloud<PointT>::Ptr cloud = boost::make_shared<pcl::PointCloud<PointT>>();
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap = cloud;

    registration->setInputTarget(globalmap);

    if (use_global_localization) {
      RCLCPP_INFO(get_logger(), "set globalmap for global localization!");
      auto req = std::make_shared<hdl_global_localization::srv::SetGlobalMap::Request>();
      pcl::toROSMsg(*globalmap, req->global_map);
      auto result = set_global_map_service->async_send_request(req);
      if (rclcpp::spin_until_future_complete(rclcpp::Node::SharedPtr(this), result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to call SetGlobalMap service");
      }
    }
  }

  /**
   * @brief perform global localization to relocalize the sensor position
   * @param
   */
  bool relocalize(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    if (last_scan == nullptr) {
      RCLCPP_INFO_STREAM(get_logger(), "no scan has been received");
      return false;
    }

    relocalizing = true;
    delta_estimater->reset();
    pcl::PointCloud<PointT>::ConstPtr scan = last_scan;

    auto query_req = std::make_shared<hdl_global_localization::srv::QueryGlobalLocalization::Request>();
    pcl::toROSMsg(*scan, query_req->cloud);
    query_req->max_num_candidates = 1;

    auto query_result_future = query_global_localization_service->async_send_request(query_req);
    if (rclcpp::spin_until_future_complete(rclcpp::Node::SharedPtr(this), query_result_future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to call QueryGlobalLocalization service");
      return false;
    }
    auto query_result = query_result_future.get();

    if (query_result->poses.empty()) {
      RCLCPP_ERROR(get_logger(), "QueryGlobalLocalization returned empty poses array");
      return false;
    }

    const auto& result = query_result->poses[0];

    RCLCPP_INFO_STREAM(get_logger(), "--- Global localization result ---");
    RCLCPP_INFO_STREAM(get_logger(), "Trans :" << result.position.x << " " << result.position.y << " " << result.position.z);
    RCLCPP_INFO_STREAM(get_logger(), "Quat  :" << result.orientation.x << " " << result.orientation.y << " " << result.orientation.z << " " << result.orientation.w);
    RCLCPP_INFO_STREAM(get_logger(), "Error :" << query_result->errors[0]);
    RCLCPP_INFO_STREAM(get_logger(), "Inlier:" << query_result->inlier_fractions[0]);

    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
    pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
    pose = pose * delta_estimater->estimated_delta();

    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    pose_estimator.reset(new hdl_localization::PoseEstimator(registration, get_clock()->now(), pose.translation(), Eigen::Quaternionf(pose.linear()), cool_time_duration));

    relocalizing = false;

    return true;
  }

  /**
   * @brief callback for initial pose input ("2D Pose Estimate" on rviz)
   * @param pose_msg
   */
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg) {
    RCLCPP_INFO(get_logger(), "initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    pose_estimator.reset(
      new hdl_localization::PoseEstimator(registration, get_clock()->now(), Eigen::Vector3f(p.x, p.y, p.z), Eigen::Quaternionf(q.w, q.x, q.y, q.z), cool_time_duration));
  }

  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if (!downsample_filter) {
      return cloud;
    }

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter->setInputCloud(cloud);
    downsample_filter->filter(*filtered);
    filtered->header = cloud->header;

    return filtered;
  }

  void publish_odometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
    // broadcast the transform over tf
    if (send_tf_transforms) {
      if (tf_buffer->canTransform(robot_odom_frame_id, odom_child_frame_id, rclcpp::Time((int64_t)0, get_clock()->get_clock_type()))) {
        geometry_msgs::msg::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
        map_wrt_frame.header.stamp = stamp;
        map_wrt_frame.header.frame_id = odom_child_frame_id;
        map_wrt_frame.child_frame_id = "map";

        geometry_msgs::msg::TransformStamped frame_wrt_odom = tf_buffer->lookupTransform(
          robot_odom_frame_id,
          odom_child_frame_id,
          rclcpp::Time((int64_t)0, get_clock()->get_clock_type()),
          rclcpp::Duration(std::chrono::milliseconds(100)));
        Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

        geometry_msgs::msg::TransformStamped map_wrt_odom;
        tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);

        tf2::Transform odom_wrt_map;
        tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
        odom_wrt_map = odom_wrt_map.inverse();

        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.transform = tf2::toMsg(odom_wrt_map);
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = robot_odom_frame_id;

        tf_broadcaster->sendTransform(odom_trans);
      } else {
        geometry_msgs::msg::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = odom_child_frame_id;
        tf_broadcaster->sendTransform(odom_trans);
      }
    }

    // publish the transform
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";

    odom.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
    // odom.pose.pose.position.x = pose_trans.transform.translation.x;
    // odom.pose.pose.position.y = pose_trans.transform.translation.y;
    // odom.pose.pose.position.z = pose_trans.transform.translation.z;
    // odom.pose.pose.orientation = pose_trans.transform.rotation;
    odom.child_frame_id = odom_child_frame_id;
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    pose_pub->publish(odom);
  }

  void publish_scan_matching_status(const std_msgs::msg::Header& header, pcl::PointCloud<pcl::PointXYZI>::ConstPtr aligned) {
    msg::ScanMatchingStatus status;
    status.header = header;

    status.has_converged = registration->hasConverged();
    status.matching_error = registration->getFitnessScore();

    const double max_correspondence_dist = 0.5;

    int num_inliers = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for (int i = 0; i < aligned->size(); i++) {
      const auto& pt = aligned->at(i);
      registration->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if (k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        num_inliers++;
      }
    }
    status.inlier_fraction = static_cast<float>(num_inliers) / aligned->size();
    status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration->getFinalTransformation().cast<double>())).transform;

    status.prediction_labels.reserve(2);
    status.prediction_errors.reserve(2);

    std::vector<double> errors(6, 0.0);

    if (pose_estimator->wo_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "without_pred";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->wo_prediction_error().get().cast<double>())).transform);
    }

    if (pose_estimator->imu_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = use_imu ? "imu" : "motion_model";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->imu_prediction_error().get().cast<double>())).transform);
    }

    if (pose_estimator->odom_prediction_error()) {
      status.prediction_labels.push_back(std_msgs::msg::String());
      status.prediction_labels.back().data = "odom";
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator->odom_prediction_error().get().cast<double>())).transform);
    }

    status_pub->publish(status);
  }

private:
  std::string robot_odom_frame_id;
  std::string odom_child_frame_id;
  bool send_tf_transforms;

  bool use_imu;
  bool invert_acc;
  bool invert_gyro;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub;
  rclcpp::Publisher<hdl_localization::msg::ScanMatchingStatus>::SharedPtr status_pub;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // imu input buffer
  std::mutex imu_data_mutex;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_data;

  // globalmap and registration method
  pcl::PointCloud<PointT>::Ptr globalmap;
  pcl::Filter<PointT>::Ptr downsample_filter;
  pcl::Registration<PointT, PointT>::Ptr registration;

  // pose estimator
  std::mutex pose_estimator_mutex;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator;

  // global localization
  bool use_global_localization;
  std::atomic_bool relocalizing;
  std::unique_ptr<DeltaEstimater> delta_estimater;

  pcl::PointCloud<PointT>::ConstPtr last_scan;
  rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedPtr set_global_map_service;
  rclcpp::Client<hdl_global_localization::srv::QueryGlobalLocalization>::SharedPtr query_global_localization_service;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_server;

  // Parameters
  double cool_time_duration;
  std::string reg_method;
  std::string ndt_neighbor_search_method;
  double ndt_neighbor_search_radius;
  double ndt_resolution;
  bool enable_robot_odometry_prediction;
};
}  // namespace hdl_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::HdlLocalizationNodelet)
