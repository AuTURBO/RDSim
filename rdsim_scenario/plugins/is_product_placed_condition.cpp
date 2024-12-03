#include "rdsim_scenario/plugins/is_product_placed_condition.hpp"

namespace rdsim_scenario {
IsProductPlacedCondition::IsProductPlacedCondition(const std::string &condition_name, const BT::NodeConfiguration &conf)
    : TopicCondition(condition_name, conf){};

bool IsProductPlacedCondition::onTick() { return topic_value_.data; }
} // namespace rdsim_scenario

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config) {
    return std::make_unique<rdsim_scenario::IsProductPlacedCondition>(name, config);
  };

  factory.registerBuilder<rdsim_scenario::IsProductPlacedCondition>("IsProductPlaced", builder);
}