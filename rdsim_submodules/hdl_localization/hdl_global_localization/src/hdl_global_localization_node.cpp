#include <iostream>
#include <boost/filesystem.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>

// #include <pcl_ros/point_cloud.h>
#include "pcl_conversions/pcl_conversions.h"
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>
#include <hdl_global_localization/srv/set_global_localization_engine.hpp>

#include <hdl_global_localization/engines/global_localization_bbs.hpp>
#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>

namespace hdl_global_localization {

class GlobalLocalizationNode : public rclcpp::Node {
public:
  GlobalLocalizationNode(const rclcpp::NodeOptions& options) : rclcpp::Node("global_localization_node", options) {
    declare_parameter<double>("globalmap_downsample_resolution", 0.5);
    declare_parameter<double>("query_downsample_resolution", 0.5);
    
    set_engine(this->declare_parameter<std::string>("global_localization_engine", std::string("FPFH_RANSAC")));
    
    set_engine_srv = create_service<hdl_global_localization::srv::SetGlobalLocalizationEngine>("set_engine", std::bind(&GlobalLocalizationNode::set_engine_cb, this, std::placeholders::_1, std::placeholders::_2));
    set_map_srv = create_service<hdl_global_localization::srv::SetGlobalMap>("set_global_map", std::bind(&GlobalLocalizationNode::set_global_map, this, std::placeholders::_1, std::placeholders::_2));
    query_localization_srv = create_service<hdl_global_localization::srv::QueryGlobalLocalization>("query", std::bind(&GlobalLocalizationNode::query, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double resolution) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxelgrid;
    voxelgrid.setLeafSize(resolution, resolution, resolution);
    voxelgrid.setInputCloud(cloud);
    voxelgrid.filter(*filtered);
    return filtered;
  }

  bool set_engine(const std::string& engine_name) {
    if (engine_name == "BBS") {
      engine.reset(new GlobalLocalizationBBS(rclcpp::Node::SharedPtr(this)));
    } else if (engine_name == "FPFH_RANSAC") {
      engine.reset(new GlobalLocalizationEngineFPFH_RANSAC(rclcpp::Node::SharedPtr(this)));
    }
    else {
      RCLCPP_ERROR_STREAM(get_logger(), "Unknown Global Localization Engine:" << engine_name);
      return false;
    }

    if (global_map) {
      engine->set_global_map(global_map);
    }

    return true;
  }

  bool set_engine_cb(srv::SetGlobalLocalizationEngine::Request::SharedPtr req, srv::SetGlobalLocalizationEngine::Response::SharedPtr res) {
    RCLCPP_INFO_STREAM(get_logger(), "Set Global Localization Engine");
    return set_engine(req->engine_name.data);
  }

  bool set_global_map(srv::SetGlobalMap::Request::SharedPtr req, srv::SetGlobalMap::Response::SharedPtr res) {
    RCLCPP_INFO_STREAM(get_logger(), "Global Map Received");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req->global_map, *cloud);
    cloud = downsample(cloud, globalmap_downsample_resolution);

    globalmap_header = req->global_map.header;
    global_map = cloud;
    engine->set_global_map(global_map);

    RCLCPP_INFO_STREAM(get_logger(), "DONE");

    return true;
  }

  bool query(srv::QueryGlobalLocalization::Request::SharedPtr req, srv::QueryGlobalLocalization::Response::SharedPtr res) {
    RCLCPP_INFO_STREAM(get_logger(), "Query Global Localization");
    if (global_map == nullptr) {
      RCLCPP_WARN_STREAM(get_logger(), "No Globalmap");
      return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req->cloud, *cloud);
    cloud = downsample(cloud, query_downsample_resolution);

    auto results = engine->query(cloud, req->max_num_candidates);

    res->inlier_fractions.resize(results.results.size());
    res->errors.resize(results.results.size());
    res->poses.resize(results.results.size());

    res->header = req->cloud.header;
    res->globalmap_header = globalmap_header;

    for (int i = 0; i < results.results.size(); i++) {
      const auto& result = results.results[i];
      Eigen::Quaternionf quat(result->pose.linear());
      Eigen::Vector3f trans(result->pose.translation());

      res->inlier_fractions[i] = result->inlier_fraction;
      res->errors[i] = result->error;
      res->poses[i].orientation.x = quat.x();
      res->poses[i].orientation.y = quat.y();
      res->poses[i].orientation.z = quat.z();
      res->poses[i].orientation.w = quat.w();

      res->poses[i].position.x = trans.x();
      res->poses[i].position.y = trans.y();
      res->poses[i].position.z = trans.z();
    }

    return !results.results.empty();
  }

private:
  rclcpp::Service<hdl_global_localization::srv::SetGlobalLocalizationEngine>::SharedPtr set_engine_srv;
  rclcpp::Service<hdl_global_localization::srv::SetGlobalMap>::SharedPtr set_map_srv;
  rclcpp::Service<hdl_global_localization::srv::QueryGlobalLocalization>::SharedPtr query_localization_srv;

  std_msgs::msg::Header globalmap_header;
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map;
  std::unique_ptr<GlobalLocalizationEngine> engine;

  double query_downsample_resolution;
  double globalmap_downsample_resolution;

};

}  // namespace hdl_global_localization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_global_localization::GlobalLocalizationNode)