cmake_minimum_required(VERSION 3.8)
project(gazebo_sfm_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)


include_directories(include)
include_directories(SYSTEM
  /usr/local/include  #to find lightsfm
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)


add_library(PedestrianSFMPlugin SHARED src/PedestrianSFMPlugin.cpp)
target_include_directories(PedestrianSFMPlugin PUBLIC include)
ament_target_dependencies(PedestrianSFMPlugin 
  "rclcpp" 
  "gazebo" 
  "gazebo_ros"
)
ament_export_libraries(PedestrianSFMPlugin)
target_link_libraries(PedestrianSFMPlugin ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES


install(TARGETS PedestrianSFMPlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)
install(DIRECTORY launch/
   DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY worlds/
   DESTINATION share/${PROJECT_NAME}/worlds
)
install(DIRECTORY media/models
   DESTINATION share/${PROJECT_NAME}/
)

ament_package()
