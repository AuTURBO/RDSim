#ifndef RDSIM__PLUGINS__IS__PRODUCT__PLACED__CONDITION_HPP_
#define RDSIM__PLUGINS__IS__PRODUCT__PLACED__CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rdsim_scenario/plugins/topic_condition.hpp"
#include "std_msgs/msg/bool.hpp"

namespace rdsim_scenario {

/**
 * @brief A BT::ConditionNode that listens to a battery topic and
 * returns SUCCESS when battery is charging and FAILURE otherwise
 */
class IsProductPlacedCondition : public TopicCondition<std_msgs::msg::Bool> {
public:
  IsProductPlacedCondition(const std::string &condition_name, const BT::NodeConfiguration &conf);
  IsProductPlacedCondition() = delete;

protected:
  bool onTick() override;
};

} // namespace rdsim_scenario

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_CHARGING_CONDITION_HPP_
