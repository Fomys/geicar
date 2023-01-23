//
// Created by andrea on 23/01/23.
//

#include "bt_plugins/new_publish_next_pose_node.hpp"

namespace nav2_behavior_tree
{

    PublishNextPose::PublishNextPose(
            const std::string &xml_tag_name,
            const BT::NodeConfiguration &conf):
            BT::SyncActionNode(xml_tag_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        std::cerr<<"Init"<<std::endl;
        RCLCPP_DEBUG(node_->get_logger(), "BIP ACTION");

        pose_publisher = node_->create_publisher<interfaces::msg::DestCmd>("dest_cmd", rclcpp::QoS(10));

    }

    BT::NodeStatus BipAction::tick()
    {
        //Info we are executing actions
        RCLCPP_DEBUG(node_->get_logger(), "Send Dest");

        //Create publisher and msg to publish
        //auto msg_dest = interfaces::msg::DestCmd();
        //msg_dest =
        //Publish on reach door

        //pose_publisher->publish(msg_dest);

        return BT::NodeStatus::SUCCESS;
    }

    void BipAction::CallbackBip(const interfaces::msg::MessageApp & msg_bip)
    {
        state = msg_bip.detect_door;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
extern "C"
{void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
    std::cerr<<"Done"<<std::endl;
    factory.registerNodeType<nav2_behavior_tree::BipAction>("PublishNextPose");
}
}
