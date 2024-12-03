#ifndef RDSIM__PLUGINS__CONDITION__TOPIC__CONDITION_HPP_
#define RDSIM__PLUGINS__CONDITION__TOPIC__CONDITION_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace rdsim_scenario {

/**
 * @brief A BT::ConditionNode that listens to a battery topic and
 * returns SUCCESS when battery is charging and FAILURE otherwise
 */
template <class TopicT> class TopicCondition : public BT::ConditionNode {
public:
  using TopicSharedPtr = std::shared_ptr<TopicT>;
  TopicCondition(const std::string &condition_name, const BT::NodeConfiguration &conf)
      : BT::ConditionNode(condition_name, conf) {
    getInput("topic_name", topic_name_);
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    topic_sub_ = node->create_subscription<TopicT>(
        topic_name_, rclcpp::SystemDefaultsQoS(),
        std::bind(&TopicCondition::topicCallback, this, std::placeholders::_1), sub_option);
  };
  TopicCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() {
    callback_group_executor_.spin_some();
    if (onTick()) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  };

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("topic_name", std::string("/topic"), "input topic name")};
  }

protected:
  virtual bool onTick() = 0;
  TopicT topic_value_;

private:
  void topicCallback(TopicSharedPtr msg) { topic_value_ = *msg; };

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  typename rclcpp::Subscription<TopicT>::SharedPtr topic_sub_;
  std::string topic_name_;
  bool is_battery_charging_;
};

} // namespace rdsim_scenario

#endif // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_BATTERY_CHARGING_CONDITION_HPP_
