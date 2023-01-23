#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NEW__BIP__ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NEW__BIP__ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/message_app.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Wait
 */
class BipAction : public BT::SyncActionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::WaitAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  BipAction(
          const std::string & xml_tag_name,
          const BT::NodeConfiguration & conf);


  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {}; //none
  }

  void CallbackBip(const interfaces::msg::MessageApp & msg_bip);

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<interfaces::msg::MessageApp>::SharedPtr bip_publisher;
  rclcpp::Subscription<interfaces::msg::MessageApp>::SharedPtr bip_subscription;

  bool state;

};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__NEW__BIP__ACTION_HPP_

