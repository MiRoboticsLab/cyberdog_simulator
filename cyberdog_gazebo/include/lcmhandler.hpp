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
#ifndef LCM_HANDLER_HPP__
#define LCM_HANDLER_HPP__

#include <lcm/lcm-cpp.hpp>

#include "simulator_lcmt.hpp"
#include "gamepad_lcmt.hpp"
#include "sim_utilities/gamepad_command.hpp" 


namespace gazebo
{
    class LCMHandler
    {
    public:
        
        /**
         * @brief Construct a new LCMHandler object
         * 
         */
        LCMHandler();
        
        /**
         * @brief Receive gamepad command messages
         * 
         * @return GamepadCommand
         */
        GamepadCommand ReceiveGPC();
        
        /**
         * @brief Send simlator state message by lcm
         * 
         * @param _lcm_sim_handler simlator state message
         */
        void SendSimData(simulator_lcmt &_lcm_sim_handler);

        /**
         * @brief Return true if lcmhandler receive a lcm message
         * 
         * @return true Lcmhandler receive a lcm message
         * @return false No lcm message is received
         */
        bool HasEvent();

    private:

        /**
         * @brief Handle gamepad command messages
         * 
         * @param rbuf 
         * @param chan 
         * @param msg 
         */
        void HandleCommand(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const gamepad_lcmt *msg);
        
        lcm::LCM lcm_;
        gamepad_lcmt lcm_gamepad_;

        GamepadCommand gamepad_command_;
    };

}

#endif //LCM_HANDLER_HPP__