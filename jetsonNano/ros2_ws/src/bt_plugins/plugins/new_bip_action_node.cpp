//
// Created by andrea on 22/01/23.
//

#include "bt_plugins/new_bip_action_node.hpp"


//Use this namespace to not have to write all the time the name
namespace nav2_behavior_tree
{

    //Constructor of a standard synchronised action node
    BipAction::BipAction(
            const std::string &xml_tag_name,
            const BT::NodeConfiguration &conf):
            BT::SyncActionNode(xml_tag_name, conf)
    {
        //Get the node of the bt to communicate with ROS
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        //Print to console that the Node has started
        std::cerr<<"bip Init"<<std::endl;
        RCLCPP_DEBUG(node_->get_logger(), "Bip init");

        //We define a subscriptor and a publisher to a topic
        rclcpp::QoS qos(rclcpp::KeepLast(10)); //Quality of service
        qos.transient_local().reliable();
        bip_publisher = node_->create_publisher<interfaces::msg::MessageApp>("/reach_door", rclcpp::QoS(10));
        bip_subscription = node_->create_subscription<interfaces::msg::MessageApp>("/reach_door", qos, std::bind(&BipAction::CallbackBip ,this, std::placeholders::_1));
    }

    //Code to execute on tick
    BT::NodeStatus BipAction::tick()
    {
        //Info we are executing actions
        RCLCPP_DEBUG(node_->get_logger(), "BIP ACTION");
        std::cerr<<"on bip"<<std::endl;

        //Create msg to publish
        auto msg_bip = interfaces::msg::MessageApp();

        //Publish on reach door
        msg_bip.detect_door = !state;
        (BipAction::bip_publisher)->publish(msg_bip); //works

        //Need to signal we have done the action
        return BT::NodeStatus::SUCCESS;
    }



    //Callback function to update variable about the state
    //¡¡DOES NOT WORK !!
    void BipAction::CallbackBip(const interfaces::msg::MessageApp & msg_bip)
    {
        std::cerr<<"get etat"<<std::endl;
        state = msg_bip.detect_door;
    }
}


//Function to register the node to the general behavior tree
//See README for explanations on the signature of the function
#include "behaviortree_cpp_v3/bt_factory.h"
extern "C"
{void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
	std::cerr<<"Done"<<std::endl;
	factory.registerNodeType<nav2_behavior_tree::BipAction>("BipAction");
}
}
