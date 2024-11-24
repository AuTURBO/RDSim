#include <mutex>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

////***************************start pyc modified***************************/
#include <std_msgs/msg/string.hpp>
using namespace std;
/***************************end pyc modified***************************////


namespace hdl_localization {

// rclcpp::Node 얘를 상속받은 GlobalmapServerNodelet class
class GlobalmapServerNodelet : public rclcpp::Node {
public:
  using PointT = pcl::PointXYZI;

  GlobalmapServerNodelet(const rclcpp::NodeOptions& options)
  // GlobalmapServerNodelet 생성자. 기본적으로 map_server 라는 이름을 갖는 인스턴스를 만듦
  : Node("map_server", options)
  {
    initialize_params();

    // publish globalmap with "latched" publisher
    auto latch_qos = rclcpp::QoS(1).transient_local();
    globalmap_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap", latch_qos);
    globalmap_pub_timer = create_wall_timer(std::chrono::milliseconds(1000), std::bind(&GlobalmapServerNodelet::pub_once_cb, this));



////***************************start pyc modified***************************/
    map_update_sub = create_subscription<std_msgs::msg::String>(
      "/map_request/pcd",
      latch_qos,
      std::bind(&GlobalmapServerNodelet::map_update_callback, this, std::placeholders::_1)
    );
/***************************end pyc modified***************************////

  }

private:

////***************************start pyc modified***************************/
  void map_update_callback(const std_msgs::msg::String::SharedPtr msg) {
    cout << "in map update callback" << endl;
    RCLCPP_INFO(get_logger(), "Received map request, map path : ");
    std::string globalmap_pcd = msg->data;
    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

    // downsample globalmap
    double downsample_resolution = declare_parameter<double>("downsample_resolution", 0.1);
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
    // sensor_msgs::msg::PointCloud2 globalmap_msg;
    // pcl::toROSMsg(*globalmap, globalmap_msg);
    // globalmap_pub->publish(globalmap_msg);
  }
/***************************end pyc modified***************************////


  void initialize_params() {

////***************************start pyc modified***************************/
    cout << endl << "initialize_params" << endl << endl;
/***************************end pyc modified***************************////

    // read globalmap from a pcd file
    std::string globalmap_pcd = declare_parameter<std::string>("globalmap_pcd", std::string(""));

////***************************start pyc modified***************************/
    cout << endl << globalmap_pcd << endl << endl;
/***************************end pyc modified***************************////

    globalmap.reset(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(globalmap_pcd, *globalmap);
    globalmap->header.frame_id = "map";

////***************************start pyc modified***************************/
    // cout << endl << *globalmap << endl << endl;
/***************************end pyc modified***************************////

    bool convert_utm_to_local = declare_parameter<bool>("convert_utm_to_local", true);
    std::ifstream utm_file(globalmap_pcd + ".utm");
    if (utm_file.is_open() && convert_utm_to_local) {
      double utm_easting;
      double utm_northing;
      double altitude;
      utm_file >> utm_easting >> utm_northing >> altitude;
      for(auto& pt : globalmap->points) {
        pt.getVector3fMap() -= Eigen::Vector3f(utm_easting, utm_northing, altitude);
      }
      RCLCPP_INFO_STREAM(get_logger(), "Global map offset by UTM reference coordinates (x = "
                      << utm_easting << ", y = " << utm_northing << ") and altitude (z = " << altitude << ")");
    }

    // downsample globalmap
    double downsample_resolution = declare_parameter<double>("downsample_resolution", 0.1);
    std::shared_ptr<pcl::VoxelGrid<PointT>> voxelgrid(new pcl::VoxelGrid<PointT>());
    voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid->setInputCloud(globalmap);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid->filter(*filtered);

    globalmap = filtered;
  }

  void pub_once_cb() {
    // sensor_msgs::msg::PointCloud2 globalmap_msg;
    globalmap_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*globalmap, *globalmap_msg);

    globalmap_pub->publish(*globalmap_msg);
    globalmap_pub_timer.reset();
  }

private:
  // ROS
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub;

  sensor_msgs::msg::PointCloud2::SharedPtr globalmap_msg;

  rclcpp::TimerBase::SharedPtr globalmap_pub_timer;
  pcl::PointCloud<PointT>::Ptr globalmap;

////***************************start pyc modified***************************/
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_update_sub;
/***************************end pyc modified***************************////

};

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::GlobalmapServerNodelet)

