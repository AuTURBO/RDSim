cmake_minimum_required(VERSION 3.8)
project(rdsim_scenario)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(BT_PATH "${CMAKE_CURRENT_BINARY_DIR}/../../install/behaviortree_cpp_v3")
set(BT_TREE_PATH "${CMAKE_CURRENT_BINARY_DIR}/../../install/nav2_behavior_tree")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED PATHS ${BT_TREE_PATH})
find_package(rdsim_interfaces REQUIRED)
find_package(behaviortree_cpp_v3 3.8.4 REQUIRED PATHS ${BT_PATH})

message(STATUS "Current source directory: ${CMAKE_CURRENT_SOURCE_DIR}")

if(behaviortree_cpp_v3_FOUND)
    if(behaviortree_cpp_v3_VERSION)
        message(WARNING "Found behaviortree_cpp_v3 version: ${behaviortree_cpp_v3_VERSION}")
    else()
        message(WARNING "behaviortree_cpp_v3 found but version information is not available.")
    endif()
else()
    message(FATAL_ERROR "behaviortree_cpp_v3 not found!")
endif()

nav2_package()

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  nav2_util
  rdsim_interfaces
  behaviortree_cpp_v3
  nav2_behavior_tree
)

add_library(${library_name}
  src/rdsim_scenario.cpp
  src/scenario/delivery_scenario.cpp
)
ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${library_name}_node
  apps/rdsim_scenario_node.cpp
)
target_link_libraries(${library_name}_node
  ${library_name}
)

install(TARGETS ${library_name}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_tutorial_test test/tutorial_test.cpp)
  target_include_directories(${PROJECT_NAME}_tutorial_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_tutorial_test
    std_msgs
  )
  target_link_libraries(${PROJECT_NAME}_tutorial_test)
endif()

ament_package()
