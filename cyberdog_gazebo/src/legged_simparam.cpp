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

#include <iostream>

#include "legged_simparam.hpp"


namespace gazebo
{
    SimParam::SimParam(std::string model_name, NodeExc* node_executor)
    :user_parameters_("user-parameters")
    {
        robotType = RobotType::MINI_CYBERDOG;

        LoadYaml();

        //build a sharedmemory with name as "development-simulator"
        printf( "[Simulation] Setup shared memory...\n" );
        shared_memory_.CreateNew( DEVELOPMENT_SIMULATOR_SHARED_MEMORY_NAME, true );
        shared_memory_.Init(true );

        shared_memory_().simToRobot.robotType  = robotType;

        gazebo_node_ = std::make_shared<GazeboNode>("gazebo_node");
        para_sub_=gazebo_node_->create_subscription<cyberdog_msg::msg::YamlParam>("yaml_parameter", 10, std::bind(&SimParam::HandleYamlParam,this,std::placeholders::_1));

        node_executor_ = node_executor;
        node_executor_->AddNode(gazebo_node_);
    }

    void SimParam::LoadYaml()
    {
        printf( "[Simulation] Loading YAML files\n" );
        user_parameters_.DefineAndInitializeFromYamlFile( GetLocoConfigDirectoryPath() + "cyberdog2-ctrl-user-parameters.yaml" );

        if ( !user_parameters_.IsFullyInitialized() ) {
        printf( "Not all user parameters were initialized. Missing:\n%s\n", user_parameters_.GenerateUnitializedList().c_str() );
        throw std::runtime_error( "not all parameters initialized from ini file" );
        }

        printf( "[Simulation] User-parameter is loaded\n" );
        robot_parameters_.InitializeFromYamlFile( GetLocoConfigDirectoryPath() + "robot-defaults.yaml" );

        if ( !robot_parameters_.IsFullyInitialized() ) {
        printf( "Not all robot control parameters were initialized. Missing:\n%s\n", robot_parameters_.GenerateUnitializedList().c_str() );
        throw std::runtime_error( "not all parameters initialized from ini file" );
        }
        printf( "[Simulation] Control-parameter is loaded\n" );

    }

    void SimParam::FirstRun()
    {
        
        shared_memory_().simToRobot.mode = SimulatorMode::DO_NOTHING;
        shared_memory_.SimulatorIsDone();

        std::cout << "[Simulation] Waiting for robot..." << std::endl;

        // this loop will check to see if the robot is connected at 10 Hz
        // doing this in a loop allows us to click the "stop" button in the GUI
        // and escape from here before the robot code connects, if needed
        bool flag_ls=false;
        while ( bool flag_ls = !shared_memory_.TryWaitForRobot() ) {
            if ( want_stop_ ) {
                return;
            }
            usleep( 100000 );
        }
        std::cout << "Success! the robot is alive" << std::endl;

        printf( "[Simulation] Send robot control parameters to robot...\n" );
        for ( auto& kv : robot_parameters_.collection_.map_ ) {
            SendControlParameter( kv.first, kv.second->Get( kv.second->kind_ ), kv.second->kind_, false );
        }

        for ( auto& kv : user_parameters_.collection_.map_ ) {
            SendControlParameter( kv.first, kv.second->Get( kv.second->kind_ ), kv.second->kind_, true );
        }
    }

    void SimParam::SendControlParameter( const std::string& name, ControlParameterValue value, ControlParameterValueKind kind, bool isUser ) {
        ControlParameterRequest&  request  = shared_memory_().simToRobot.controlParameterRequest;
        ControlParameterResponse& response = shared_memory_().robotToSim.controlParameterResponse;

        // first check no pending message
        assert( request.requestNumber == response.requestNumber );

        // new message
        request.requestNumber++;

        // message data
        request.requestKind = isUser ? ControlParameterRequestKind::kSET_USER_PARAM_BY_NAME : ControlParameterRequestKind::kSET_ROBOT_PARAM_BY_NAME;
        strcpy( request.name, name.c_str() );
        request.value         = value;
        request.parameterKind = kind;
        printf( "%s\n", request.ToString().c_str() );

        // run robot:
        shared_memory_().simToRobot.mode = SimulatorMode::RUN_CONTROL_PARAMETERS;
        shared_memory_.SimulatorIsDone();

        // wait for robot code to finish
        if ( shared_memory_.WaitForRobotWithTimeout() ) {
        }
        else {
            HandleControlError();
            request.requestNumber = response.requestNumber;  // so if we come back we won't be off by 1
            return;
        }

        //shared_memory_().waitForRobot();

        // verify response is good
        assert( response.requestNumber == request.requestNumber );
        assert( response.parameterKind == request.parameterKind );
        assert( std::string( response.name ) == request.name );
    }

    void SimParam::HandleControlError() {
        want_stop_  = true;
        running_   = false;
        connected_ = false;
        if ( !shared_memory_().robotToSim.errorMessage[ 0 ] ) {
            printf( "[ERROR] Control code timed-out!\n" );
            error_callback_( "Control code has stopped responding without giving an error message.\nIt has likely crashed - "
                            "check the output of the control code for more information" );
        }
        else {
            printf( "[ERROR] Control code has an error!\n" );
            error_callback_( "Control code has an error:\n" + std::string( shared_memory_().robotToSim.errorMessage ) );
        }
    }

    void SimParam::SendSMData(SimulatorToRobotMessage _SimToRobot)
    {     
        if (shared_memory_().simToRobot.mode != SimulatorMode::EXIT)
        {
            shared_memory_().simToRobot.cheaterState = _SimToRobot.cheaterState;
            shared_memory_().simToRobot.spiData = _SimToRobot.spiData;
            shared_memory_().simToRobot.vectorNav = _SimToRobot.vectorNav;
            shared_memory_().simToRobot.mode = SimulatorMode::RUN_CONTROLLER;
            if(lcm_has_event_)
            {
                shared_memory_().simToRobot.gamepadCommand = _SimToRobot.gamepadCommand;
                lcm_has_event_ = false;
            }
            shared_memory_.SimulatorIsDone();

        }
        if ( shared_memory_.WaitForRobotWithTimeout() ) {
        }
        else {
            HandleControlError();
            return;
        }


    }

    SpiCommand SimParam::ReceiveSMData()
    {
        SpiCommand _spicommand;
        _spicommand = shared_memory_().robotToSim.spiCommand;
        return _spicommand;
    }

    void SimParam::HandleYamlParam(const cyberdog_msg::msg::YamlParam::SharedPtr msg)
    {
        ParamHandler topic_paramhandler_;
        topic_paramhandler_.name = msg->name;

        switch (msg->kind)
        {
        case cyberdog_msg::msg::YamlParam::DOUBLE:
            topic_paramhandler_.kind = ControlParameterValueKind::kDOUBLE;
            topic_paramhandler_.value.d = msg->double_value;
            break;
        
        case cyberdog_msg::msg::YamlParam::S64:
            topic_paramhandler_.kind = ControlParameterValueKind::kS64;
            topic_paramhandler_.value.i = msg->s64_value;
            
            break;
        
        case cyberdog_msg::msg::YamlParam::VEC_X_DOUBLE:
            topic_paramhandler_.kind = ControlParameterValueKind::kVEC_X_DOUBLE;
            for(int i=0; i<12; i++)
                {
                    topic_paramhandler_.value.vecXd[i]=msg->vecxd_value[i];
                }
                
            break;

        default:
            break;
        }

        bool isUser_ = false;

        if(msg->is_user) {
            isUser_=true;
        }

        SendControlParameter(topic_paramhandler_.name, topic_paramhandler_.value, topic_paramhandler_.kind, isUser_);

    }

}