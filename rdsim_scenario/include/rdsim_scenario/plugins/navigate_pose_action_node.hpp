#ifndef __RDSIM_NAVIGATE_POSE_ACTION_NODE_H__
#define __RDSIM_NAVIGATE_POSE_ACTION_NODE_H__

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace rdsim_scenario {
class NavigatePoseActionNode : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose> {
public:
  NavigatePoseActionNode(const std::string &xml_tag_name, const std::string &action_name,
                         const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<std::string>("behavior_tree", "Behavior tree to run"),
    });
  }
};
} // namespace rdsim_scenario

#endif // __RDSIM_NAVIGATE_POSE_ACTION_NODE_H__