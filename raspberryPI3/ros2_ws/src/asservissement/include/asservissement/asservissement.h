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
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/angle_order.hpp"

//STD
#include "std_srvs/srv/empty.hpp"

//Defines
#define PERIOD_UPDATE_PARAM 1000ms //to update params every second





#endif //DIRECTION_CONTROL_DIRECTION_CONTROL_NODE_H
