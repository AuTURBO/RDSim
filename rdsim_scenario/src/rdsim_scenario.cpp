#include "rdsim_scenario/rdsim_scenario.hpp"

namespace rdsim_scenario {
ScenarioManager::ScenarioManager() : nav2_util::LifecycleNode("rdsim_scenario", "") {
  m_delivery_scenario = std::make_unique<DeliveryScenario>();
}

nav2_util::CallbackReturn ScenarioManager::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  std::vector<std::string> plugin_lib_names = {
      "nav2_recovery_node_bt_node",           "nav2_pipeline_sequence_bt_node",
      "nav2_round_robin_node_bt_node",        "nav2_navigate_through_poses_action_bt_node",
      "nav2_navigate_to_pose_action_bt_node", "nav2_is_battery_charging_condition_bt_node"};
  if (!m_delivery_scenario->on_configure(shared_from_this(), plugin_lib_names, &m_muxer)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ScenarioManager::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Activating");

  if (!m_delivery_scenario->on_activate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ScenarioManager::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (!m_delivery_scenario->on_deactivate()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ScenarioManager::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  if (!m_delivery_scenario->on_cleanup()) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  m_delivery_scenario.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn ScenarioManager::on_shutdown(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

} // namespace rdsim_scenario