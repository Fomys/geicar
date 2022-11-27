//
// Created by Madeline on 27/11/2022.
//

#ifndef GEICAR_REPLAY_H
#define GEICAR_REPLAY_H


/*
 * Includes for cpp file
 */

//General
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
//Messages

#include "interfaces/msg/angle_order.hpp"
#include "interfaces/msg/speed_input.hpp"
#include "interfaces/msg/scenario_to_play.hpp"


//STD
#include "std_srvs/srv/empty.hpp"

//Defines
#define PERIOD_UPDATE_CMD 1ms //to execute cmd by brain every ms


#endif //GEICAR_REPLAY_H
