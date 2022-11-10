#include "../include/asservissement/asservissement.h"

using namespace std;
using placeholders::_1;

class asservissement : public rclcpp::Node{

public:
    asservissement():Node("asservissement_node")
    {

        //Parameter declaration for gains in PID
        this->declare_parameter("kp_l", 1.0);
        this->declare_parameter("ki_l", 1.0);
        this->declare_parameter("kd_l", 1.0);

        this->declare_parameter("kp_r", 1.0);
        this->declare_parameter("ki_r", 1.0);
        this->declare_parameter("kd_r", 1.0);

        this->declare_parameter("kp_s", 1.0);
        this->declare_parameter("ki_s", 1.0);
        this->declare_parameter("kd_s", 1.0);

        //Set value of the parameter for use
        Kp_l = this->get_parameter("kp_l").get_parameter_value().get<float>();
        Ki_l = this->get_parameter("ki_l").get_parameter_value().get<float>();
        Kd_l = this->get_parameter("kd_l").get_parameter_value().get<float>();

        Kp_r = this->get_parameter("kp_r").get_parameter_value().get<float>();
        Ki_r = this->get_parameter("ki_r").get_parameter_value().get<float>();
        Kd_r = this->get_parameter("kd_r").get_parameter_value().get<float>();

        Kp_s = this->get_parameter("kp_s").get_parameter_value().get<float>();
        Ki_s = this->get_parameter("ki_s").get_parameter_value().get<float>();
        Kd_s = this->get_parameter("kd_s").get_parameter_value().get<float>();

        //Publishers
        // sur le topic commande des moteurs
        publisher_can_ = this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        //Subscriptions
        // Données des moteurs
        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
                "motors_feedback", 10, std::bind(&asservissement::motorsFeedbackCallback, this, _1));

        //Users input
        subscription_brain_order_ = this->create_subscription<interfaces::msg::AngleOrder>(
                "brain_order", 10, std::bind(&asservissement::executeAngleCmd, this, _1));

        //Timer for update of parameters
        timer_ = this->create_wall_timer(PERIOD_UPDATE_PARAM, std::bind(&asservissement::updateParameters, this));


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

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Parameter declarations
    //left wheel PID corrector parameters
    float Kp_l;
    float Ki_l;
    float Kd_l;
    //right wheel PID corrector parameters
    float Kp_r;
    float Ki_r;
    float Kd_r;
    //steering PID correcto paramers
    float Kp_s;
    float Ki_s;
    float Kd_s;


    /*
     * Callback to execute the angle cmd of the brain when order is sent
     */
    void executeAngleCmd(const interfaces::msg::AngleOrder & angle)
    {

    }

    /*
     * Callback function when feedback topic is updated
     */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback)
    {

    }

    /*
     * Update Parameter values every second in case of a change
     */
    void updateParameters()
    {
        Kp_l = this->get_parameter("kp_l").get_parameter_value().get<float>();
        Ki_l = this->get_parameter("ki_l").get_parameter_value().get<float>();
        Kd_l = this->get_parameter("kd_l").get_parameter_value().get<float>();

        Kp_r = this->get_parameter("kp_r").get_parameter_value().get<float>();
        Ki_r = this->get_parameter("ki_r").get_parameter_value().get<float>();
        Kd_r = this->get_parameter("kd_r").get_parameter_value().get<float>();

        Kp_s = this->get_parameter("kp_s").get_parameter_value().get<float>();
        Ki_s = this->get_parameter("ki_s").get_parameter_value().get<float>();
        Kd_s = this->get_parameter("kd_s").get_parameter_value().get<float>();

        //Print all to log
//        RCLCPP_INFO(this->get_logger(), "Valeur Kp l : %f", Kp_l);
//        RCLCPP_INFO(this->get_logger(), "Valeur Ki l : %f", Ki_l);
//        RCLCPP_INFO(this->get_logger(), "Valeur Kd l : %f", Kd_l);
//        RCLCPP_INFO(this->get_logger(), "Valeur Kp r : %f", Kp_r);
//        RCLCPP_INFO(this->get_logger(), "Valeur Ki r : %f", Ki_r);
//        RCLCPP_INFO(this->get_logger(), "Valeur Kd r : %f", Kd_r);
//        RCLCPP_INFO(this->get_logger(), "Valeur Kp s : %f", Kp_s);
//        RCLCPP_INFO(this->get_logger(), "Valeur Ki s : %f", Ki_s);
//        RCLCPP_INFO(this->get_logger(), "Valeur Kd s : %f", Kd_s);
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