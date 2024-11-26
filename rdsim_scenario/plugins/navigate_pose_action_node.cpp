#include "rdsim_scenario/plugins/navigate_pose_action_node.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config) {
    return std::make_unique<rdsim_scenario::NavigatePoseActionNode>(name, config);
  };

  factory.registerBuilder<rdsim_scenario::NavigatePoseActionNode>("ComputeGoalToTopology", builder);
}