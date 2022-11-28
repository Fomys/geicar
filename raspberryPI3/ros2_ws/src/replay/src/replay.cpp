#include "../include/replay/replay.h"

using namespace std;
using placeholders::_1;

class replay : public rclcpp::Node {

public:
    replay() : Node("replay_node") {


        //Publishers
        // on the SpeedInput topic (need to pass through the security node)
        publisher_speed_ = this->create_publisher<interfaces::msg::SpeedInput>("speed_input", 10);

        // on the AngleOrder topic
        publisher_steering_ = this->create_publisher<interfaces::msg::AngleOrder>("angle_order", 10);

        //Subscriptions

        //Scenario to replay order (from interface)
        subscription_scenarioToPlay_ = this->create_subscription<interfaces::msg::ScenarioToPlay>(
                "scenarioToPlay", 10, std::bind(&replay::scenarioToPlayCallback, this, _1));

        timer_cmd_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&replay::executeReplay, this));

        //Inform the log the node has been launched
        RCLCPP_INFO(this->get_logger(), "replay_node READY");
    }

private:

    /************ VARIABLE DECLARATIONS *******************/

    //**ROS TOPIC & TIMERS DECLARATION**
    //Publishers
    rclcpp::Publisher<interfaces::msg::SpeedInput>::SharedPtr publisher_speed_;
    rclcpp::Publisher<interfaces::msg::AngleOrder>::SharedPtr publisher_steering_;
    //Subscriptions
    rclcpp::Subscription<interfaces::msg::ScenarioToPlay>::SharedPtr subscription_scenarioToPlay_;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_cmd_;


    //**VARIABLES FOR FILES MANAGEMENT

    bool finishedPlay = true;
    ifstream fichier_enregistrement;
    bool problem = false;



    /************ FUNCTION DECLARATIONS *******************/

    /*
    * If replay mode, update speed and steering command by reading the scenario file every 1ms
    */
    void executeReplay() {
        auto angle_order = interfaces::msg::AngleOrder();
        auto speed_input = interfaces::msg::SpeedInput();
        if (!finishedPlay) {
            if (fichier_enregistrement) {
                string line;
                if (getline(fichier_enregistrement, line)) {
                    //On lit une ligne complète
                    istringstream iss(line);
                    vector <string> results((istream_iterator<string>(iss)), istream_iterator<string>());
                    speed_input.speed_order_input = stof(results[0]);
                    angle_order.angle_order = stof(results[1]);
                    publisher_speed_->publish(speed_input);
                    publisher_steering_->publish(angle_order);
                } else {
                    finishedPlay = true;
                    RCLCPP_INFO(this->get_logger(), "End of the scenario");
                }

            } else {
                if (!problem) {
                    problem = true;
                    RCLCPP_INFO(this->get_logger(), "ERROR: Cannot open file in read mode");
                }
            }
        }
    }

        //** CALLBACK FUNCTIONS TO UPDATE INTERNAL VARIABLES **

        /*
         * Callback to update scenario to play
         */
        void scenarioToPlayCallback(const interfaces::msg::ScenarioToPlay &scenarioToPlay) {
                finishedPlay = false;
                problem = false;
                fichier_enregistrement.open(scenarioToPlay.scenario_file, std::ifstream::in);
                RCLCPP_INFO(this->get_logger(), "Launching the scenario");
                //RCLCPP_INFO(this->get_logger(), "Launching of the scenario written in %s", scenarioToPlay.scenario_file);

        }


};


//Running node
    int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<replay>();

        rclcpp::spin(node);

        rclcpp::shutdown();
        return 0;
    }


