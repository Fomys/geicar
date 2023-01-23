
#include "../include/asservissement/asservissement.h"

using namespace std;
using placeholders::_1;

class asservissement : public rclcpp::Node{

public:
    asservissement():Node("asservissement_node")
    {
        //Parameter declaration for gains in PID
        //PID of the motor of the wheels
        this->declare_parameter("kp", 0.8);
        this->declare_parameter("ki", 900.0);
        this->declare_parameter("kd", 0.0);

        //Set value of the parameter for use
        Kp = this->get_parameter("kp").get_parameter_value().get<float>();
        Ki = this->get_parameter("ki").get_parameter_value().get<float>();
        Kd = this->get_parameter("kd").get_parameter_value().get<float>();

        //Publishers
        // sur le topic commande des moteurs
        publisher_can_ = this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        //Subscriptions
        // Données des moteurs
        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
                "motors_feedback", 10, std::bind(&asservissement::motorsFeedbackCallback, this, _1));

        subscription_speed_order_ = this->create_subscription<interfaces::msg::SpeedOrder>(
                "speed_order", 10, std::bind(&asservissement::UpdateCmdSpeedOrder, this, _1));

        //Timer for update of parameters
        timer_parameter_ = this->create_wall_timer(PERIOD_UPDATE_PARAM, std::bind(&asservissement::updateParameters, this));
        timer_cmd_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&asservissement::executeCmd, this));

        //Inform the log the node has been launched
        RCLCPP_INFO(this->get_logger(), "asservissement_node READY");
    }

private:

    /************ VARIABLE DECLARATIONS *******************/

    //**ROS TOPIC & TIMERS DECLARATION**
    //Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;

    //Subscriptions
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::SpeedOrder>::SharedPtr subscription_speed_order_;
    //Timer
    rclcpp::TimerBase::SharedPtr timer_parameter_;
    rclcpp::TimerBase::SharedPtr timer_cmd_;


    //**VARIABLES FOR CONTROL**
    //Requested commands by brain
    float requestedSteerAngle = 0;
    float requestedSpeed = 0;

    //Data from motor feedback
    float currentAngle ;
    float currentRightRearSpeed;
    float currentLeftRearSpeed;

    //For PID Calculations
    float I_x;

    rclcpp::Time time_last = this->get_clock()->now();

    //**Control variables**
    uint8_t PwmCmd;
    uint8_t steeringPwmCmd;

    //**PARAMETER DECLARATIONS**
    //wheel PID corrector parameters
    float Kp;
    float Ki;
    float Kd;

    /************ FUNCTION DECLARATIONS *******************/

    //** CALLBACK FUNCTIONS TO UPDATE INTERNAL VARIABLES **

    /*
     * Callback to update the velocity command value
     */
    void UpdateCmdSpeedOrder(const interfaces::msg::SpeedOrder & speed_order)
    {
        requestedSpeed = speed_order.speed_order ;
        requestedSteerAngle = speed_order.angle_order;

        RCLCPP_INFO(this->get_logger(), "La vitesse qui vient d'etre demandee est %f RPM", requestedSpeed);
    }



    /*
     * Callback function when feedback topic is updated
     */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback)
    {
        currentAngle = motorsFeedback.steering_angle;
        currentRightRearSpeed = motorsFeedback.right_rear_speed;
        currentLeftRearSpeed = motorsFeedback.left_rear_speed;
    }

    /*
     * Update Parameter values every second in case of a change
     */
    void updateParameters()
    {
        Kp = this->get_parameter("kp").get_parameter_value().get<float>();
        Ki = this->get_parameter("ki").get_parameter_value().get<float>();
        Kd = this->get_parameter("kd").get_parameter_value().get<float>();
    }

    /*
     * Function executing the PID controller every PERIOD_UPDATE_CMD = 1ms currently
     */
    void executeCmd()
    {
        auto motorsOrder = interfaces::msg::MotorsOrder();
        //Asservissement roues
        asservSpeed();
        motorsOrder.left_rear_pwm = PwmCmd;
        motorsOrder.right_rear_pwm = PwmCmd;
        //Asservissement steering
        asservSteering();
        motorsOrder.steering_pwm = steeringPwmCmd;
        publisher_can_->publish(motorsOrder);
    }


    void asservSpeed ()
    {
        float speedError;
        float PwmCmd_computed;

        //Termes proportionnels pour moteur droit et gauche
        float P_x;

        //Computation of the error for Kp
        if (requestedSpeed >= 0)
            speedError = requestedSpeed - (currentRightRearSpeed + currentLeftRearSpeed)/2;
        else
            speedError = requestedSpeed + (currentRightRearSpeed + currentLeftRearSpeed)/2;

        // Terme proportionnel
        P_x = speedError * Kp;

        // Terme intégral
        rclcpp::Duration dt(this->get_clock()->now() - time_last);
        double delta_t = dt.seconds()*0.001;
        I_x = I_x + Ki * delta_t * speedError;
        time_last = this->get_clock()->now();

        // Calcul de la commande
        PwmCmd_computed = P_x + I_x;

        if ( requestedSpeed >= 0)
        {
            if (PwmCmd_computed < 0)
                PwmCmd_computed = 0;
            else if (PwmCmd_computed > 50)
                PwmCmd_computed = 50;
            //Set the offset, because cmd = [0 : 50] goes backwards
            // And cmd = [50 : 100] goes forwards
            PwmCmd_computed += 50;
        }
        else if (requestedSpeed < 0)
        {
            if (PwmCmd_computed > 0)
                PwmCmd_computed = 0;
            else if (PwmCmd_computed < -50)
                PwmCmd_computed = -50;
            //Set the offset, because cmd = [0 : 50] goes backwards
            // And cmd = [50 : 100] goes forwards
            PwmCmd_computed += 50;
        }
        PwmCmd = PwmCmd_computed;
    }

    void asservSteering ()
    {
       //Computation of the error for Kp
        float errorAngle = currentAngle - requestedSteerAngle;

        //Command's calculation
        if (abs(errorAngle)<TOLERANCE_ANGLE){
            steeringPwmCmd = STOP;
        }
        else {
            if (errorAngle>0) {
                steeringPwmCmd = MAX_PWM_LEFT;
            }
            else {
                steeringPwmCmd = MAX_PWM_RIGHT;
            }
        }
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
