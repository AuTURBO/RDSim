cmake_minimum_required(VERSION 3.5)
project(hdl_global_localization)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetGlobalLocalizationEngine.srv
  srv/SetGlobalMap.srv
  srv/QueryGlobalLocalization.srv
DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

add_library(${PROJECT_NAME}_lib SHARED
  src/hdl_global_localization/bbs/bbs_localization.cpp
  src/hdl_global_localization/ransac/voxelset.cpp
  src/hdl_global_localization/ransac/ransac_pose_estimation.cpp
  src/hdl_global_localization/engines/global_localization_bbs.cpp
  src/hdl_global_localization/engines/global_localization_fpfh_ransac.cpp
  src/hdl_global_localization_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  pcl_ros
  geometry_msgs
  nav_msgs
)
rosidl_target_interfaces(${PROJECT_NAME}_lib
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)
target_include_directories(
  ${PROJECT_NAME}_lib PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
)
rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "${PROJECT_NAME}::GlobalLocalizationNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(
  TARGETS
    ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
