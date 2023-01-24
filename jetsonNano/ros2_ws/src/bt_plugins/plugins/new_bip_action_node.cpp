//
// Created by andrea on 22/01/23.
//

#include "bt_plugins/new_bip_action_node.hpp"

namespace nav2_behavior_tree
{

    BipAction::BipAction(
            const std::string &xml_tag_name,
            const BT::NodeConfiguration &conf):
            BT::SyncActionNode(xml_tag_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        std::cerr<<"bip Init"<<std::endl;
        RCLCPP_DEBUG(node_->get_logger(), "Bip init");

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.transient_local().reliable();
        bip_publisher = node_->create_publisher<interfaces::msg::MessageApp>("/reach_door", rclcpp::QoS(10));
        bip_subscription = node_->create_subscription<interfaces::msg::MessageApp>("/reach_door", qos, std::bind(&BipAction::CallbackBip ,this, std::placeholders::_1));
    }

    BT::NodeStatus BipAction::tick()
    {
        //Info we are executing actions
        RCLCPP_DEBUG(node_->get_logger(), "BIP ACTION");
        std::cerr<<"on bip"<<std::endl;
        //Create publisher and msg to publish
        auto msg_bip = interfaces::msg::MessageApp();

        //Publish on reach door
        msg_bip.detect_door = !state;
        (BipAction::bip_publisher)->publish(msg_bip);

        return BT::NodeStatus::SUCCESS;
    }

    void BipAction::CallbackBip(const interfaces::msg::MessageApp & msg_bip)
    {
        std::cerr<<"get etat"<<std::endl;
        state = msg_bip.detect_door;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
extern "C"
{void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
	std::cerr<<"Done"<<std::endl;
	factory.registerNodeType<nav2_behavior_tree::BipAction>("BipAction");
}
}
