// Copyright (c) 2021 Samsung Research
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rdsim_scenario/scenario/delivery_scenario.hpp"
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace rdsim_scenario {

bool DeliveryScenario::configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) {
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  if (!node->has_parameter("start_blackboard_id")) {
    node->declare_parameter("start_blackboard_id", std::string("start_goal"));
  }

  start_goal_blackboard_id_ = node->get_parameter("start_blackboard_id").as_string();

  if (!node->has_parameter("end_blackboard_id")) {
    node->declare_parameter("end_blackboard_id", std::string("end_goal"));
  }

  end_goal_blackboard_id_ = node->get_parameter("end_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }

  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());
  return true;
}

std::string DeliveryScenario::getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node) {
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_nav_to_pose_bt_xml")) {
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("rdsim_scenario");
    node->declare_parameter<std::string>("default_nav_to_pose_bt_xml", pkg_share_dir + "/behavior_trees/delivery.xml");
  }

  node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool DeliveryScenario::cleanup() {
  goal_sub_.reset();
  self_client_.reset();
  return true;
}

bool DeliveryScenario::goalReceived(ActionT::Goal::ConstSharedPtr goal) {
  auto bt_xml_filename = goal->behavior_tree;
  RCLCPP_INFO(logger_, "bt_name: %s", bt_xml_filename.c_str());

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(logger_, "BT file not found");
    return false;
  }

  initializeGoalPose(goal);

  return true;
}

void DeliveryScenario::goalCompleted(typename ActionT::Result::SharedPtr /*result*/,
                                     const nav2_behavior_tree::BtStatus /*final_bt_status*/) {}

void DeliveryScenario::onLoop() {
  // action server feedback (pose, duration of task,
  // number of recoveries, and distance remaining to goal)
}

void DeliveryScenario::onPreempt(ActionT::Goal::ConstSharedPtr goal) {
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
      (goal->behavior_tree.empty() &&
       bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename())) {
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializeGoalPose(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(logger_, "Preemption request was rejected since the requested BT XML file is not the same "
                         "as the one that the current goal is executing. Preemption with a new BT is invalid "
                         "since it would require cancellation of the previous goal instead of true preemption."
                         "\nCancel the current goal and send a new action request if you want to use a "
                         "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

void DeliveryScenario::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal) {
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0); // NOLINT

  // // Update the goal pose on the blackboard
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  blackboard->set<geometry_msgs::msg::PoseStamped>("initial_goal", pose);
  blackboard->set<geometry_msgs::msg::PoseStamped>(start_goal_blackboard_id_, goal->start_pose);
  blackboard->set<geometry_msgs::msg::PoseStamped>(end_goal_blackboard_id_, goal->end_pose);
}

} // namespace rdsim_scenario
