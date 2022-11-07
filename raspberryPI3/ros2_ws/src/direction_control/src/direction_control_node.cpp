#include "../include/direction_control/direction_control_node.h"

using namespace std;
using placeholders::_1;

class direction_control_node : public rclcpp::Node{

public:
    direction_control_node():Node("direction_control_node")
    {
        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
                "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "direction_control_node READY");
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