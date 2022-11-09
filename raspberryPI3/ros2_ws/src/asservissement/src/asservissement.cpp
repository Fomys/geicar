#include "../include/asservissement/asservissement.h"

using namespace std;
using placeholders::_1;

class asservissement : public rclcpp::Node{

public:
    asservissement():Node("asservissement_node")
    {

        //Parameter declaration for gains in PID
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{}; //description of param
        param_desc.description = "Gain Kp";
        this->declare_parameter("kp", 1.0, param_desc);

        //Set value of the parameter for use
        Kp = this->get_parameter("kp").get_parameter_value().get<float>();

        //Publishers
        // sur le topic commande des moteurs
        publisher_can_ = this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        //Subscriptions
        // Données des moteurs
        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
                "motors_feedback", 10, std::bind(&asservissement::motorsFeedbackCallback, this, _1));

//        //Users input
        subscription_brain_order_ = this->create_subscription<interfaces::msg::AngleOrder>(
                "brain_order", 10, std::bind(&asservissement::executeAngleCmd, this, _1));


        //Inform the log the node has been launched
        RCLCPP_INFO(this->get_logger(), "asservissement_node READY");
    }

private:

    //Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;

    //Subscriptions
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::AngleOrder>::SharedPtr subscription_brain_order_;

    //Requested angle by brain
    int requestedSteerAngle = 0;

    //Data for motor feedback
    uint8_t steeringPwmCmd;

    //Data about PID corrector
    int Kp = 1;

    /*
     * Callback to execute the angle cmd of the brain when order is sent
     */
    void executeAngleCmd(const interfaces::msg::AngleOrder & angle)
    {
        RCLCPP_INFO(this->get_logger(), "Valeur Kp : %d", Kp);
    }

    /*
     * Callback function when feedback topic is updated
     */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback)
    {

    }

};


//Running node
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<asservissement>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}