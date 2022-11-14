#include <stdint.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <chrono>
#include <functional>
#include <string>
#include <iostream>
#include <sstream>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/can_status.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/ultrasonic.hpp"

#include "../include/can/can.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class CanNode : public rclcpp::Node {
public:
    CanNode()
            : Node("can_node") {
      auto can_interface_param_description = rcl_interfaces::msg::ParameterDescriptor{};
      can_interface_param_description.description = "Can interface to use.";
      this->declare_parameter("can_interface", "vcan0", can_interface_param_description);

      auto auto_reconnect_param_description = rcl_interfaces::msg::ParameterDescriptor{};
      auto_reconnect_param_description.description = "Enable to reconnect can bus after a failure.";
      this->declare_parameter("auto_reconnect", false, auto_reconnect_param_description);

      auto can_timeout_param_description = rcl_interfaces::msg::ParameterDescriptor{};
      can_timeout_param_description.description = "Can timeout (in s)";
      this->declare_parameter("can_timeout", 1, can_timeout_param_description);

      auto stm32f103_timeout_param_description = rcl_interfaces::msg::ParameterDescriptor{};
      stm32f103_timeout_param_description.description = "STM32F103 timeout (in s)";
      this->declare_parameter("stm32f103_timeout", 5, stm32f103_timeout_param_description);

      publisher_can_status_ = this->create_publisher<interfaces::msg::CanStatus>("can_status", 1);
      publisher_motors_feedback_ = this->create_publisher<interfaces::msg::MotorsFeedback>("motors_feedback", 1);
      publisher_ultrasound_front_ = this->create_publisher<interfaces::msg::Ultrasonic>("ultrasonic_front", 1);
      publisher_ultrasound_back_ = this->create_publisher<interfaces::msg::Ultrasonic>("ultrasonic_back", 1);

      subscription_motors_order_ = this->create_subscription<interfaces::msg::MotorsOrder>(
              "motors_order", 1,
              std::bind(&CanNode::cb_send_motors_order, this, _1));

      this->status_timer_ = this->create_wall_timer(500ms, std::bind(&CanNode::cb_status_timer, this));

      this->can_connect();
      this->read_thread_ = std::thread(&CanNode::read_thread, this);
    }

    ~CanNode() {
      this->stopping = true;
      this->can_close();
      this->read_thread_.join();
    }

private:
    int socket_;
    bool socket_connected_;
    int stm32f103_last_response_;

    std::thread read_thread_;
    bool stopping;

    rclcpp::Publisher<interfaces::msg::Ultrasonic>::SharedPtr publisher_ultrasound_front_;
    rclcpp::Publisher<interfaces::msg::Ultrasonic>::SharedPtr publisher_ultrasound_back_;
    rclcpp::Publisher<interfaces::msg::CanStatus>::SharedPtr publisher_can_status_;
    rclcpp::Publisher<interfaces::msg::MotorsFeedback>::SharedPtr publisher_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::MotorsOrder>::SharedPtr subscription_motors_order_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    void read_thread() {
      ssize_t bytes_read;
      struct can_frame frame;

      size_t pending_buffer_content = 0;
      char pending_buffer[sizeof(struct can_frame)];
      char recv_buffer[sizeof(struct can_frame)];

      while(!this->stopping) {
        // condition de socket fonctionnel
        if(this->socket_connected_) {
          // Try to read missing data
          bytes_read = read(this->socket_, &recv_buffer, sizeof(pending_buffer) - pending_buffer_content);
          // Handle read errors
          if(bytes_read < 0) {
            if(errno != EAGAIN) {
              RCLCPP_ERROR_STREAM(this->get_logger(), "CAN read failed: read error : " << strerror(errno));
              this->can_close();
            }
          } else {
            // Copy received data to temporary buffer
            memcpy(&pending_buffer[pending_buffer_content], &recv_buffer, bytes_read);
            pending_buffer_content += bytes_read;

            // If the buffer is full, copy it to a real frame and handle it
            if(pending_buffer_content == sizeof(struct can_frame)) {
              memcpy(&frame, pending_buffer, sizeof(struct can_frame));
              pending_buffer_content = 0;
              this->can_handle_frame(frame);
            }
          }
        } else {
          this->can_connect();
        }
      }
    }

    void can_handle_frame(struct can_frame frame) {
      std::ostringstream data;
      for(int i = 0; i < frame.can_dlc; i++) {
        data << "0x" << std::hex << (int) frame.data[i] << " ";
      }
      RCLCPP_DEBUG_STREAM(this->get_logger(), "CAN receive ID: 0x" << std::hex << frame.can_id << " Len: " << std::dec << frame.can_dlc << " Data: " << data.str());

      if (frame.can_id == ID_MOTORS_DATAS) {
        can_handle_motors_data(frame);
      } else if (frame.can_id == ID_US_FRONT) {
        can_handle_ultrasound_front(frame);
      } else if (frame.can_id == ID_US_BACK) {
        can_handle_ultrasound_back(frame);
      }
    }

    void can_handle_motors_data(struct can_frame frame) {
      if(frame.can_dlc != 7) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Invalid motor frame received, ignore it.");
        return;
      }

      auto motors_feedback = interfaces::msg::MotorsFeedback();

      // Update odometry
      motors_feedback.left_rear_odometry = (int8_t) frame.data[0];
      motors_feedback.right_rear_odometry = (int8_t) frame.data[1];

      // Update Motors speed
      uint16_t leftSpeedMes = (frame.data[2] << 8) + frame.data[3];
      uint16_t rightSpeedMes = (frame.data[4] << 8) + frame.data[5];

      motors_feedback.left_rear_speed = 0.01 * leftSpeedMes ;
      motors_feedback.right_rear_speed = 0.01 * rightSpeedMes ;

      //Update Steering Angle
      uint8_t steer = frame.data[6];  //Receive steer in [0;200]
      motors_feedback.steering_angle = (steer - 100.0) / 100.0; //Convert steer in [-1;1]

      //Publication on topics
      this->stm32f103_last_response_ = time(nullptr);
      publisher_motors_feedback_->publish(motors_feedback);
    }

    void can_handle_ultrasound_front(struct can_frame frame) {
      if(frame.can_dlc != 6) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Invalid ultrasound front frame received, ignore it.");
        return;
      }

      auto ultrasound = interfaces::msg::Ultrasonic();
      ultrasound.left = (frame.data[0] << 8) | frame.data[1];
      ultrasound.center = (frame.data[2] << 8) | frame.data[3];
      ultrasound.right = (frame.data[4] << 8) | frame.data[5];
      this->stm32f103_last_response_ = time(nullptr);
      this->publisher_ultrasound_front_->publish(ultrasound);
    }

    void can_handle_ultrasound_back(struct can_frame frame) {
      if(frame.can_dlc != 6) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Invalid ultrasound back frame received, ignore it.");
        return;
      }

      auto ultrasound = interfaces::msg::Ultrasonic();
      ultrasound.left = (frame.data[0] << 8) | frame.data[1];
      ultrasound.center = (frame.data[2] << 8) | frame.data[3];
      ultrasound.right = (frame.data[4] << 8) | frame.data[5];
      this->stm32f103_last_response_ = time(nullptr);
      this->publisher_ultrasound_back_->publish(ultrasound);
    }

    void can_handle_battery(struct can_frame frame) {
      auto battery = interfaces::msg::BatteryMessage();
      
    }

    void can_connect() {
      struct ifreq ifr;
      struct sockaddr_can addr;

      // Create the socket for the CAN
      this->socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
      if (this->socket_ < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "CAN initialization failed: socket creation error : " << strerror(errno));
        this->can_close();
        return;
      }

      // Set read timeout
      struct timeval tv;
      tv.tv_usec = 0;
      tv.tv_sec = this->get_parameter("can_timeout").get_parameter_value().get<int>();
      if(setsockopt(this->socket_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv) < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CAN initialization failed: setsockopt error : " << strerror(errno));
        this->can_close();
        return;
      }

      strcpy(ifr.ifr_name, this->get_parameter("can_interface").get_parameter_value().get<std::string>().c_str());
      if (ioctl(this->socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CAN initialization failed: ioctl error : " << strerror(errno));
        this->can_close();
        return;
      }

      memset(&addr, 0, sizeof(addr));
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;

      if (bind(this->socket_, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "CAN initialization failed: bind error : " << strerror(errno));
        this->can_close();
        return;
      }

      RCLCPP_INFO_STREAM(this->get_logger(), "CAN initialized successfully");
      this->socket_connected_ = true;
    }

    void can_close() {
      if(this->socket_ >= 0) {
        if(close(this->socket_) < 0) {
          RCLCPP_FATAL_STREAM(this->get_logger(), "CAN de-initialization failed: close error : " << strerror(errno));
          this->socket_connected_ = false;
          return;
        }
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "CAN de-initialization successfully");
      this->socket_connected_ = false;
      this->socket_ = -1;
      return;
    }

    void cb_send_motors_order(const interfaces::msg::MotorsOrder &motorsOrder) {
      struct can_frame frame;

      frame.can_dlc = 8;
      frame.can_id = ID_MOTORS_CMD;

      frame.data[0] = motorsOrder.left_rear_pwm;
      frame.data[1] = motorsOrder.right_rear_pwm;
      frame.data[2] = motorsOrder.steering_pwm;

      this->can_send(frame);
    }

    void can_send(struct can_frame frame) {
      if(!this->socket_connected_ && this->get_parameter("auto_reconnect").get_parameter_value().get<bool>()) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Can bus disconnected, trying to reconnect before sending new message");
        this->can_connect();
      }

      if(this->socket_connected_) {
        if (write(this->socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "Sending fail, can ID 0x" << std::hex << frame.can_id << ": write : " << strerror(errno));
          this->can_close();
          return;
        }

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Sending success, can ID 0x" << std::hex << frame.can_id);
        return;
      }
    }

    void cb_status_timer() {
      auto can_status = interfaces::msg::CanStatus();
      can_status.socket_connected = this->socket_connected_;
      can_status.stm32f103_connected = time(nullptr) - this->stm32f103_last_response_ < this->get_parameter("stm32f103_timeout").get_parameter_value().get<int>();
      this->publisher_can_status_->publish(can_status);
    }
};


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanNode>());
  rclcpp::shutdown();
  return 0;
}