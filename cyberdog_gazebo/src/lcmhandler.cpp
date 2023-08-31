// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lcmhandler.hpp"

namespace gazebo
{
    LCMHandler::LCMHandler(){
        if (!lcm_.good()){
         exit(1);
       }

    lcm_.subscribe("gamepad_lcmt", &LCMHandler::HandleCommand, this);

    }

    GamepadCommand LCMHandler::ReceiveGPC()
    {
        lcm_.handle();
        gamepad_command_.a=lcm_gamepad_.a;
        gamepad_command_.b=lcm_gamepad_.b;
        gamepad_command_.x=lcm_gamepad_.x;
        gamepad_command_.y=lcm_gamepad_.y;
        gamepad_command_.leftStickAnalog[0]=lcm_gamepad_.leftStickAnalog[0];
        gamepad_command_.leftStickAnalog[1]=lcm_gamepad_.leftStickAnalog[1];
        gamepad_command_.rightStickAnalog[0]=lcm_gamepad_.rightStickAnalog[0];
        gamepad_command_.rightStickAnalog[1]=lcm_gamepad_.rightStickAnalog[1];

        return gamepad_command_;
    }

    void LCMHandler::HandleCommand(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const gamepad_lcmt *msg)
    {
        lcm_gamepad_ = *msg;
    }

    bool LCMHandler::HasEvent()
    {
        // setup the LCM file descriptor for waiting.
        int lcm_fd = lcm_.getFileno();
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(lcm_fd, &fds);

        // wait a limited amount of time for an incoming message
        struct timeval timeout = {
            0, // seconds
            1  // microseconds
        };
        int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

        if (0 == status) {
        // no messages
        return false;
        }
        else if (FD_ISSET(lcm_fd, &fds)) {
        // LCM has events ready to be processed.
        return true;
        }
        else{
        // Undefined
        return false;
        }
    }

    void LCMHandler::SendSimData(simulator_lcmt &_lcm_sim_handler)
    {
        lcm_.publish("simulator_state", &_lcm_sim_handler);
    }

}