//
// Created by andrea on 23/01/23.
//

#include "bt_plugins/new_wait_package_node.hpp"

namespace nav2_behavior_tree
{

    WaitPackage::WaitPackage(
            const std::string &xml_tag_name,
            const BT::NodeConfiguration &conf):
            BT::ConditionNode(xml_tag_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.transient_local().reliable();
        package_subscription = node_->create_subscription<interfaces::msg::Package>("/detect_package", qos, std::bind(&WaitPackage::CallbackWaitPackage ,this, std::placeholders::_1));

        std::cerr<<package_subscription->get_topic_name()<<std::endl;
        std::cerr<<package_subscription->get_subscription_handle()<<std::endl;

        getInput("fetch_drop", drop);

        std::cerr<<"Init Package"<<std::endl;
        RCLCPP_DEBUG(node_->get_logger(), "Init Package");


    }

    BT::NodeStatus WaitPackage::tick()
    {
        //Info we are executing actions
        RCLCPP_DEBUG(node_->get_logger(), "tick package");
        //std::cerr<<"wait package"<<std::endl;
        //We have to drop a package, we wait for true on detection
        //We have to fetch a package, we wait for false on detection
        if (state == drop) {
            std::cerr<<"YES"<<std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else {
            std::cerr<<"no"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }

    }


    void
    WaitPackage::CallbackWaitPackage(const interfaces::msg::Package & msg_package)
    {
        //Get state of the package
        std::cerr<<"get_state"<<std::endl;
        state = msg_package.state_pack;
    }


}

#include "behaviortree_cpp_v3/bt_factory.h"
extern "C"
{void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
    std::cerr<<"Done"<<std::endl;
    factory.registerNodeType<nav2_behavior_tree::WaitPackage>("WaitPackage");
}
}

