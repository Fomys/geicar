//
// Created by andrea on 07/11/22.
//

#ifndef DIRECTION_CONTROL_DIRECTION_CONTROL_NODE_H
#define DIRECTION_CONTROL_DIRECTION_CONTROL_NODE_H

/*
 * Includes for cpp file
 */

//General
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>

//Messages
#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
//#include "interfaces/msg/angle_order.hpp"
//#include "interfaces/msg/speed_order.hpp"

#include "geometry_msgs/msg/twist.hpp"

//STD
#include "std_srvs/srv/empty.hpp"

//Defines
#define PERIOD_UPDATE_PARAM 1000ms //to update params every second
#define PERIOD_UPDATE_CMD 10ms //to execute cmd by brain every ms
#define STOP 50
#define MAX_PWM_LEFT 0
#define MAX_PWM_RIGHT 100
#define TOLERANCE 2.0
#define TOLERANCE_ANGLE 0.04    // Previous value  0.07



#endif //DIRECTION_CONTROL_DIRECTION_CONTROL_NODE_H
