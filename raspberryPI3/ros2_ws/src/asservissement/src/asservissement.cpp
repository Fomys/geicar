#include "../include/asservissement/asservissement.h"

using namespace std;
using placeholders::_1;

class asservissement : public rclcpp::Node{

public:
    asservissement():Node("asservissement_node")
    {

        //Parameter declaration for gains in PID
        //PID of the motor of the left wheel
        this->declare_parameter("kp_l", 1.0);
        this->declare_parameter("ki_l", 0.01);
        this->declare_parameter("kd_l", 0.0);
        //PID of the motor of the right wheel
        this->declare_parameter("kp_r", 1.0);
        this->declare_parameter("ki_r", 0.01);
        this->declare_parameter("kd_r", 0.0);
        //PID of the motor of direction
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
        subscription_angle_order_ = this->create_subscription<interfaces::msg::AngleOrder>(
                "angle_order", 10, std::bind(&asservissement::UpdateCmdAngle, this, _1));
        subscription_speed_order_ = this->create_subscription<interfaces::msg::SpeedOrder>(
                "speed_order", 10, std::bind(&asservissement::UpdateCmdSpeed, this, _1));

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
    rclcpp::Subscription<interfaces::msg::AngleOrder>::SharedPtr subscription_angle_order_;
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
    float sumIntegralLeft;
    float sumIntegralRight;
    //float sumIntegralSteerAngle;       // New for Steering
    float previousSpeedErrorLeft;
    float previousSpeedErrorRight;
    //float previousSteerAngle;       // New for Steering

    //**Control variables**
    uint8_t leftRearPwmCmd;
    uint8_t rightRearPwmCmd;
    uint8_t steeringPwmCmd;
    float cmd_RearSpeed;


    //**PARAMETER DECLARATIONS**
    //left wheel PID corrector parameters
    float Kp_l;
    float Ki_l;
    float Kd_l;
    //right wheel PID corrector parameters
    float Kp_r;
    float Ki_r;
    float Kd_r;
    //steering PID corrector parameters
    float Kp_s;
    float Ki_s;
    float Kd_s;


    /************ FUNCTION DECLARATIONS *******************/

    //** CALLBACK FUNCTIONS TO UPDATE INTERNAL VARIABLES **

    /*
     * Callback to update the angle command value by
     */
    void UpdateCmdAngle(const interfaces::msg::AngleOrder & angle)
    {
        requestedSteerAngle = angle.angle_order;
        RCLCPP_INFO(this->get_logger(), "Valeur Angle : %f", requestedSteerAngle);
    }


    void UpdateCmdSpeed(const interfaces::msg::SpeedOrder & speed)
    {
        requestedSpeed  = speed.speed_order;
        //RCLCPP_INFO(this->get_logger(), "Valeur Speed : %f", requestedSpeed);
    }

    /*
     * Callback function when feedback topic is updated
     */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback)
    {
        currentAngle = motorsFeedback.steering_angle;
        currentRightRearSpeed = motorsFeedback.right_rear_speed;
        currentLeftRearSpeed = motorsFeedback.left_rear_speed;
//        RCLCPP_INFO(this->get_logger(), "Valeur currentAngle : %f", currentAngle);
//        RCLCPP_INFO(this->get_logger(), "Valeur currentRightRearSpeed : %f", currentRightRearSpeed);
//        RCLCPP_INFO(this->get_logger(), "Valeur currentLeftRearSpeed : %f", currentLeftRearSpeed);
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

    /*
     * Function executing the PID controller every PERIOD_UPDATE_CMD = 1ms currently
     */
    void executeCmd()
    {
        auto motorsOrder = interfaces::msg::MotorsOrder();
        //Asservissement roues
        asservSpeed();
        motorsOrder.left_rear_pwm = leftRearPwmCmd;
        motorsOrder.right_rear_pwm = rightRearPwmCmd;
        //Asservissement steering
        asservSteering();
        motorsOrder.steering_pwm = steeringPwmCmd;
        publisher_can_->publish(motorsOrder);
    }


    void asservSpeed ()
    {
        float speedErrorLeft;
        float speedErrorRight;

        float deltaErrorLeft;
        float deltaErrorRight ;

        float leftPwmCmd;
        float rightPwmCmd;

        //Computation of the error for Kp
        speedErrorLeft = requestedSpeed - currentLeftRearSpeed;
        speedErrorRight = requestedSpeed - currentRightRearSpeed;

        //Computation of the error for Ki
        sumIntegralLeft += speedErrorLeft;
        sumIntegralRight += speedErrorRight;

        //Computation of the error for Kd
        deltaErrorLeft = speedErrorLeft - previousSpeedErrorLeft;
        deltaErrorRight = speedErrorRight - previousSpeedErrorRight;
        previousSpeedErrorLeft = speedErrorLeft;
        previousSpeedErrorRight = speedErrorRight;

        //Computation of the command that must be sent to the motors
        leftPwmCmd = speedErrorLeft * Kp_l + sumIntegralLeft * Ki_l + deltaErrorLeft * Kd_l;
        rightPwmCmd = speedErrorRight * Kp_r + sumIntegralRight * Ki_r + deltaErrorRight * Kd_r;

        //In order to avoid breaking the motor,
        // we prevent the motors to spin backwards (The command must be greater than 50.)
        if(leftPwmCmd < 0)
            leftPwmCmd = 0;
        else if(leftPwmCmd > 50)
            leftPwmCmd = 50;

        if(rightPwmCmd < 0)
            rightPwmCmd = 0;
        else if(rightPwmCmd > 50)
            rightPwmCmd = 50;

        //Set the offset, because cmd = [0 : 50] goes backwards
        // And cmd = [50 : 100] goes forwards
        leftPwmCmd += 50;
        rightPwmCmd += 50;
        leftRearPwmCmd = leftPwmCmd;
        rightRearPwmCmd = rightPwmCmd;
    }

    void asservSteering ()
    {

        //float steerPwmCmd;
        //float deltaErrorSteer;

        //Computation of the error for Kp_s
        float errorAngle = currentAngle - requestedSteerAngle;

        //Computation of the error for Ki_s
        //sumIntegralSteerAngle += errorAngle;

        //Computation of the error for Kd_s
        //deltaErrorSteer = errorAngle - previousSteerAngle;

        //Computation of the command for the steering angle
        //steerPwmCmd = errorAngle * Kp_s + sumIntegralSteerAngle * Ki_s + deltaErrorSteer * Kd_s;

        //steeringPwmCmd = steerPwmCmd;

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