#include <hdl_global_localization/engines/global_localization_bbs.hpp>

#include "pcl_conversions/pcl_conversions.h"
#include <sensor_msgs/msg/point_cloud.hpp>

#include <hdl_global_localization/bbs/bbs_localization.hpp>
#include <hdl_global_localization/bbs/occupancy_gridmap.hpp>

namespace hdl_global_localization {

GlobalLocalizationBBS::GlobalLocalizationBBS(rclcpp::Node::SharedPtr node) : node(node) {
  gridmap_pub = this->node->create_publisher<nav_msgs::msg::OccupancyGrid>("gridmap", 1);
  map_slice_pub = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("map_slice", 1);
  scan_slice_pub = this->node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_slice", 1);

  params.max_range = this->node->declare_parameter<double>("bbs/max_range", 15.0);
  params.min_tx = this->node->declare_parameter<double>("bbs/min_tx", -10.0);
  params.max_tx = this->node->declare_parameter<double>("bbs/max_tx", 10.0);
  params.min_ty = this->node->declare_parameter<double>("bbs/min_ty", -10.0);
  params.max_ty = this->node->declare_parameter<double>("bbs/max_ty", 10.0);
  params.min_theta = this->node->declare_parameter<double>("bbs/min_theta", -3.15);
  params.max_theta = this->node->declare_parameter<double>("bbs/max_theta", 3.15);
  params.map_min_z = this->node->declare_parameter<double>("bbs/map_min_z", 2.0);
  params.map_max_z = this->node->declare_parameter<double>("bbs/map_max_z", 2.4);
  params.map_resolution = this->node->declare_parameter<double>("bbs/map_resolution", 0.5);
  params.scan_min_z = this->node->declare_parameter<double>("bbs/scan_min_z", -0.2);
  params.scan_max_z = this->node->declare_parameter<double>("bbs/scan_max_z", 0.2);
  params.map_width = this->node->declare_parameter<int>("bbs/map_width", 1024);
  params.map_height = this->node->declare_parameter<int>("bbs/map_height", 1024);
  params.map_pyramid_level = this->node->declare_parameter<int>("bbs/map_pyramid_level", 6);
  params.max_points_per_cell = this->node->declare_parameter<int>("bbs/max_points_per_cell", 5);
}

GlobalLocalizationBBS ::~GlobalLocalizationBBS() {}

void GlobalLocalizationBBS::set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  BBSParams p;
  p.max_range = params.max_range;
  p.min_tx = params.min_tx;
  p.max_tx = params.max_tx;
  p.min_ty = params.min_ty;
  p.max_ty = params.max_ty;
  p.min_theta = params.min_theta;
  p.max_theta = params.max_theta;
  bbs.reset(new BBSLocalization(p));

  auto map_2d = slice(*cloud, params.map_min_z, params.map_max_z);
  RCLCPP_INFO_STREAM(node->get_logger(), "Set Map " << map_2d.size() << " points");

  if (map_2d.size() < 128) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Num points in the sliced map is too small!!");
    RCLCPP_WARN_STREAM(node->get_logger(), "Change the slice range parameters!!");
  }

  bbs->set_map(map_2d, params.map_resolution, params.map_width, params.map_height, params.map_pyramid_level, params.max_points_per_cell);

  auto map_3d = unslice(map_2d);
  map_3d->header.frame_id = "map";
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*map_3d, msg);
  map_slice_pub->publish(msg);
  gridmap_pub->publish(*bbs->gridmap()->to_rosmsg());
}

GlobalLocalizationResults GlobalLocalizationBBS::query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) {
  auto scan_2d = slice(*cloud, params.scan_min_z, params.scan_max_z);

  std::vector<GlobalLocalizationResult::Ptr> results;

  RCLCPP_INFO_STREAM(node->get_logger(), "Query " << scan_2d.size() << " points");
  if (scan_2d.size() < 32) {
    RCLCPP_WARN_STREAM(node->get_logger(), "Num points in the sliced scan is too small!!");
    RCLCPP_WARN_STREAM(node->get_logger(), "Change the slice range parameters!!");
    return GlobalLocalizationResults(results);
  }

  double best_score = 0.0;
  auto trans_2d = bbs->localize(scan_2d, 0.0, &best_score);
  if (trans_2d == boost::none) {
    return GlobalLocalizationResults(results);
  }

  if (scan_slice_pub->get_subscription_count()) {
    auto scan_3d = unslice(scan_2d);
    scan_3d->header = cloud->header;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*scan_3d, msg);
    scan_slice_pub->publish(msg);
  }

  Eigen::Isometry3f trans_3d = Eigen::Isometry3f::Identity();
  trans_3d.linear().block<2, 2>(0, 0) = trans_2d->linear();
  trans_3d.translation().head<2>() = trans_2d->translation();

  results.resize(1);
  results[0].reset(new GlobalLocalizationResult(best_score, best_score, trans_3d));

  return GlobalLocalizationResults(results);
}

GlobalLocalizationBBS::Points2D GlobalLocalizationBBS::slice(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_z, double max_z) const {
  Points2D points_2d;
  points_2d.reserve(cloud.size());
  for (int i = 0; i < cloud.size(); i++) {
    if (min_z < cloud.at(i).z && cloud.at(i).z < max_z) {
      points_2d.push_back(cloud.at(i).getVector3fMap().head<2>());
    }
  }
  return points_2d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GlobalLocalizationBBS::unslice(const Points2D& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->resize(points.size());
  for (int i = 0; i < points.size(); i++) {
    cloud->at(i).getVector3fMap().head<2>() = points[i];
    cloud->at(i).z = 0.0f;
  }

  return cloud;
}

}  // namespace hdl_global_localization