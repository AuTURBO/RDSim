#ifndef __RDSIM_SCENARIO_H__
#define __RDSIM_SCENARIO_H__

#include "nav2_behavior_tree/bt_action_server.hpp"
#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace rdsim_scenario {

class ScenarioMuxer {
public:
  /**
   * @brief A Navigator Muxer constructor
   */
  ScenarioMuxer() : current_navigator_(std::string("")) {}

  /**
   * @brief Get the navigator muxer state
   * @return bool If a navigator is in progress
   */
  bool isNavigating() {
    std::scoped_lock l(mutex_);
    return !current_navigator_.empty();
  }

  /**
   * @brief Start navigating with a given navigator
   * @param string Name of the navigator to start
   */
  void startNavigating(const std::string &navigator_name) {
    std::scoped_lock l(mutex_);
    if (!current_navigator_.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigatorMutex"), "Major error! Navigation requested while another navigation"
                                                         " task is in progress! This likely occurred from an incorrect"
                                                         "implementation of a navigator plugin.");
    }
    current_navigator_ = navigator_name;
  }

  /**
   * @brief Stop navigating with a given navigator
   * @param string Name of the navigator ending task
   */
  void stopNavigating(const std::string &navigator_name) {
    std::scoped_lock l(mutex_);
    if (current_navigator_ != navigator_name) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigatorMutex"), "Major error! Navigation stopped while another navigation"
                                                         " task is in progress! This likely occurred from an incorrect"
                                                         "implementation of a navigator plugin.");
    } else {
      current_navigator_ = std::string("");
    }
  }

protected:
  std::string current_navigator_;
  std::mutex mutex_;
};

template <class ActionT> class Scenario {
public:
  using Ptr = std::shared_ptr<rdsim_scenario::Scenario<ActionT>>;

  Scenario() { plugin_muxer_ = nullptr; };
  virtual ~Scenario() = default;

  bool on_configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
                    const std::vector<std::string> &plugin_lib_names, rdsim_scenario::ScenarioMuxer *plugin_muxer) {
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    plugin_muxer_ = plugin_muxer;

    // get the default behavior tree for this Scenario
    std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);
    int16_t debug_port = 6666;

    // Create the Behavior Tree Action Server for this Scenario
    bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
        node, getName(), plugin_lib_names, default_bt_xml_filename,
        std::bind(&Scenario::onGoalReceived, this, std::placeholders::_1), std::bind(&Scenario::onLoop, this),
        std::bind(&Scenario::onPreempt, this, std::placeholders::_1),
        std::bind(&Scenario::onCompletion, this, std::placeholders::_1, std::placeholders::_2), debug_port);

    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      ok = false;
    }

    // BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();

    return configure(parent_node) && ok;
  }

  /**
   * @brief Activation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool on_activate() {
    bool ok = true;

    if (!bt_action_server_->on_activate()) {
      ok = false;
    }

    return activate() && ok;
  }

  /**
   * @brief Deactivation of the navigator's backend BT and actions
   * @return bool If successful
   */
  bool on_deactivate() {
    bool ok = true;
    if (!bt_action_server_->on_deactivate()) {
      ok = false;
    }

    return deactivate() && ok;
  }

  /**
   * @brief Cleanup a navigator
   * @return bool If successful
   */
  bool on_cleanup() {
    bool ok = true;
    if (!bt_action_server_->on_cleanup()) {
      ok = false;
    }

    bt_action_server_.reset();

    return cleanup() && ok;
  }

  /**
   * @brief Get the action name of this navigator to expose
   * @return string Name of action to expose
   */
  virtual std::string getName() = 0;

  virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;

  /**
   * @brief Get the action server
   * @return Action server pointer
   */
  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> &getActionServer() { return bt_action_server_; }

protected:
  /**
   * @brief An intermediate goal reception function to mux navigators.
   */
  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal) {
    if (plugin_muxer_->isNavigating()) {
      RCLCPP_ERROR(logger_,
                   "Requested navigation from %s while another navigator is processing,"
                   " rejecting request.",
                   getName().c_str());
      return false;
    }

    bool goal_accepted = goalReceived(goal);

    if (goal_accepted) {
      plugin_muxer_->startNavigating(getName());
    }

    return goal_accepted;
  }

  /**
   * @brief An intermediate completion function to mux navigators
   */
  void onCompletion(typename ActionT::Result::SharedPtr result, const nav2_behavior_tree::BtStatus final_bt_status) {
    plugin_muxer_->stopNavigating(getName());
    goalCompleted(result, final_bt_status);
  }

  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   */
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  virtual void onLoop() = 0;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that is called when a the action is completed; Can fill in
   * action result message or indicate that this action is done.
   */
  virtual void goalCompleted(typename ActionT::Result::SharedPtr result,
                             const nav2_behavior_tree::BtStatus final_bt_status) = 0;

  /**
   * @param Method to configure resources.
   */
  virtual bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr /*node*/) { return true; }

  /**
   * @brief Method to cleanup resources.
   */
  virtual bool cleanup() { return true; }

  /**
   * @brief Method to activate any threads involved in execution.
   */
  virtual bool activate() { return true; }

  /**
   * @brief Method to deactivate and any threads involved in execution.
   */
  virtual bool deactivate() { return true; }

protected:
  std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("Scenario")};
  rclcpp::Clock::SharedPtr clock_;
  ScenarioMuxer *plugin_muxer_;
};
} // namespace rdsim_scenario

#endif // __DELIVERY_SCENARIO_H__