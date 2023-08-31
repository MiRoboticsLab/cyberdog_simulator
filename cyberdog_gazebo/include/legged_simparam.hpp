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
#ifndef _LEGGED_SIMPARAM_HPP__
#define _LEGGED_SIMPARAM_HPP__

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <cyberdog_msg/msg/yaml_param.hpp>

#include "ctrl_ros/cpp_types.hpp"
#include "utilities/shared_memory.hpp"
#include "sim_utilities/simulator_message.hpp"

#include "ctrl_ros/control_parameters/control_parameters.hpp"
#include "ctrl_ros/control_parameters/robot_parameters.hpp"
#include "node_executor.hpp"

namespace gazebo
{
    struct ParamHandler
    {
        std::string name;
        ControlParameterValueKind kind;
        ControlParameterValue value;
    };
    
    class SimParam
    {
    public: 
        /**
         * @brief Construct a new Sim Param object
         * 
         * @param model_name name of robot
         * @param node_executor node executor to subscribe yaml message
         */
        SimParam(std::string model_name, NodeExc* node_executor);

        /**
         * @brief Build connection to control program at the first run
         * 
         */
        void FirstRun();

        /**
         * @brief Send sharedmemory data to control program
         * 
         * @param _SimToRobot sharedmemory data to control program
         */
        void SendSMData(SimulatorToRobotMessage _SimToRobot);

        /**
         * @brief Receive sharedmemory data from control program
         * 
         * @return SpiCommand sharedmemory data from control program
         */
        SpiCommand ReceiveSMData();

        /**
         * @brief Receive YamlParam topic message
         * 
         */
        void ReceiveTopic();

        /**
         * @brief Set the LcmHasEvent if gamepad lcm message is received
         * 
         * @return true Gamepad lcm message is received
         * @return false No gamepad lcm message is received
         */
        void LcmHasEvent(){lcm_has_event_ = true;};
        
    private:

        /**
         * @brief Load simulator and control parameter from yaml files
         * 
         */
        void LoadYaml();

        /**
         * @brief Send ControlParameter by shared memory to control program
         * 
         * @param name Name of ControlParameter
         * @param value Value of ControlParameter
         * @param kind Kind of ControlParameter
         * @param isUser true for userparameter
         *               false for robotparameter
         */
        void SendControlParameter( const std::string& name, ControlParameterValue value, ControlParameterValueKind kind, bool isUser );

        /**
         * @brief Handle error message if control program has a error
         * 
         */
        void HandleControlError();

        /**
         * @brief Handle YamlParam topic message
         * 
         * @param msg YamlParam topic message
         */
        void HandleYamlParam(const cyberdog_msg::msg::YamlParam::SharedPtr msg);


        SharedMemoryObject<SimulatorMessage>    shared_memory_;
    
        RobotType robotType;
        ControlParameters                       user_parameters_;
        RobotControlParameters                  robot_parameters_;
        bool                                    lcm_has_event_;

        // ros2 node to receive YamlParam topic
        std::shared_ptr<GazeboNode> gazebo_node_;
        rclcpp::Subscription<cyberdog_msg::msg::YamlParam>::SharedPtr para_sub_;

        NodeExc*      node_executor_ =   nullptr;

        std::function< void( std::string ) >    error_callback_;
        bool                                    running_                    = false;
        bool                                    connected_                  = false;
        bool                                    want_stop_                   = false;

    };
}

#endif //_LEGGED_SIMPARAM_HPP__