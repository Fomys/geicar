#include "../include/direction_control/direction_control_node.h"

using namespace std;
using placeholders::_1;

class direction_control_node : public rclcpp::Node{

public:
    direction_control_node():Node("direction_control_node")
    {

        //Parameter declaration for gains in PID
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; //description of param
        param_desc.description = "Gain Kp";
        this->declare_parameter("kp", 1, param_desc);

        //Set value of the parameter for use
        int kp = this->get_parameter("kp").get_parameter_value();

        //Publisher sur le topic commande des moteurs
        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        //Subscription aux données des moteurs
        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
                "motors_feedback", 10, std::bind(&car_control::updateData, this, _1));

        //Subscription to the users input
        subscription_brain_order_ = this->create_subscription<interfaces::msg::MovementOrder>(
                "brain_order", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        //Inform the log the node has been launched
        RCLCPP_INFO(this->get_logger(), "direction_control_node READY");
    }

private:

    //Requested angle by brain
    int requestedSteerAngle = 0;

    //Data for motor feedback
    uint8_t steeringPwmCmd;

    //Execute the Cmd of the brain when an order is sent
    void executeBrainCmd(const interfaces::msg::MotorsOrder & order)
    {
        RCLCPP_INFO(this->get_logger(), "Valeur Kp : %d", this->get_parameter("kp").get_parameter_value());
    }






};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<car_control>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}