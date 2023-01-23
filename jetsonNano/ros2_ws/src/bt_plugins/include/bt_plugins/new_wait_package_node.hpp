//
// Created by andrea on 23/01/23.
//

#ifndef ROS2_WS_NEW_WAIT_ROS_TOPIC_NODE_HPP
#define ROS2_WS_NEW_WAIT_ROS_TOPIC_NODE_HPP

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/package.hpp"


namespace nav2_behavior_tree
{

class WaitPackage : public BT::SyncActionNode
{
public:
    /**
     * @brief A constructor for nav2_behavior_tree::WaitAction
     * @param xml_tag_name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    WaitPackage(
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
        return {
            BT::InputPort<bool>("fetch_drop", "Indicate if we have to drop(true) or fetch(false) a package")
        }; //none
    }

    void CallbackWaitPackage(const interfaces::msg::Package & msg_package);

    rclcpp::Node::SharedPtr node_;

    bool drop;

    bool state;

    rclcpp::Subscription<interfaces::msg::Package>::SharedPtr package_subscription;

};

}  // namespace nav2_behavior_tree

#endif //ROS2_WS_NEW_WAIT_ROS_TOPIC_NODE_HPP
