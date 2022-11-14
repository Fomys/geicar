#include <rclcpp/rclcpp.hpp>
//#include <unistd.h>
//#include <functional>
//#include <memory>
//#include <string>

#include "interfaces/msg/status.hpp"
#include "interfaces/msg/can_status.hpp"

#include "../include/system_check/system_check_node.hpp"

using namespace std;
using placeholders::_1;


class SystemStatus : public rclcpp::Node {

public:
    SystemStatus()
            : Node("system_status_node") {
      auto can_timeout_param_description = rcl_interfaces::msg::ParameterDescriptor{};
      can_timeout_param_description.description = "Can timeout (in ms)";
      this->declare_parameter("can_timeout", (time_t) 5000, can_timeout_param_description);


      publisher_status_ = this->create_publisher<interfaces::msg::Status>("status", 1);

      subscription_can_status_ = this->create_subscription<interfaces::msg::CanStatus>(
              "can_status", 1,
              std::bind(&SystemStatus::cb_can_status, this, _1));

      this->status_timer_ = this->create_wall_timer(500ms, std::bind(&SystemStatus::cb_status_timer, this));
    }

    void cb_status_timer() {
      time_t now = time(nullptr);
      auto status = interfaces::msg::Status();
      if(now - this->last_can_status > this->get_parameter("can_timeout").get_parameter_value().get<time_t>()) {
        status.can_socket_connected = interfaces::msg::Status::CAN_SOCKET_NOT_RUNNING;
      } else if(this->can_status) {
        status.can_socket_connected = interfaces::msg::Status::CAN_SOCKET_CONNECTED;
      } else {
        status.can_socket_connected = interfaces::msg::Status::CAN_SOCKET_DISCONNECTED;
      }
      status.stm32f103_connected = this->stm32f103_connected;
      this->publisher_status_->publish(status);
    }

    void cb_can_status(const interfaces::msg::CanStatus &can_status) {
      this->last_can_status = time(nullptr);
      this->can_status = can_status.socket_connected;
      this->stm32f103_connected = can_status.stm32f103_connected;
    }

private:
    time_t last_can_status;
    bool can_status;
    bool stm32f103_connected;


    rclcpp::Publisher<interfaces::msg::Status>::SharedPtr publisher_status_;
    rclcpp::Subscription<interfaces::msg::CanStatus>::SharedPtr subscription_can_status_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SystemStatus>());
  rclcpp::shutdown();
  return 0;
}