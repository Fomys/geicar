//
// Created by andrea on 23/01/23.
//

#ifndef ROS2_WS_NEW_PUBLISH_NEXT_POSE_NODE_HPP
#define ROS2_WS_NEW_PUBLISH_NEXT_POSE_NODE_HPP

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/message_app.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps nav2_msgs::action::Wait
 */
    class PublishNextPose : public BT::SyncActionNode
    {
    public:
        /**
         * @brief A constructor for nav2_behavior_tree::WaitAction
         * @param xml_tag_name Name for the XML tag for this node
         * @param conf BT node configuration
         */
        PublishNextPose(
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

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<interfaces::msg::MessageApp>::SharedPtr pose_publisher;

    };

}  // namespace nav2_behavior_tree


#endif //ROS2_WS_NEW_PUBLISH_NEXT_POSE_NODE_HPP
