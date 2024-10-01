#ifndef HDL_GLOBAL_LOCALIZATION_BBS_HPP
#define HDL_GLOBAL_LOCALIZATION_BBS_HPP

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <hdl_global_localization/engines/global_localization_engine.hpp>

namespace hdl_global_localization {

typedef struct {
  double max_range;
  double min_tx;
  double max_tx;
  double min_ty;
  double max_ty;
  double min_theta;
  double max_theta;
  double map_min_z;
  double map_max_z;
  double map_resolution;
  double scan_min_z;
  double scan_max_z;
  int map_width;
  int map_height;
  int map_pyramid_level;
  int max_points_per_cell;
} GlobalBBSParams;

class BBSLocalization;

class GlobalLocalizationBBS : public GlobalLocalizationEngine {
public:
  GlobalLocalizationBBS(rclcpp::Node::SharedPtr node);
  virtual ~GlobalLocalizationBBS() override;

  virtual void set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) override;

  virtual GlobalLocalizationResults query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) override;

private:
  using Points2D = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;
  Points2D slice(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_z, double max_z) const;

  pcl::PointCloud<pcl::PointXYZ>::Ptr unslice(const Points2D& points);

protected:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr gridmap_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_slice_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_slice_pub;

  std::unique_ptr<BBSLocalization> bbs;
  GlobalBBSParams params;
  rclcpp::Node::SharedPtr node;
};

}  // namespace hdl_global_localization

#endif